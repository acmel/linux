/*
 * builtin-htop.c
 *
 * hist_entry based top.
 */
#include "builtin.h"

#include "util/util.h"

#include "util/color.h"
#include <linux/list.h>
#include "util/cache.h"
#include <linux/rbtree.h>
#include "util/symbol.h"
#include "util/strlist.h"
#include "util/values.h"

#include "perf.h"
#include "util/debug.h"
#include "util/cpumap.h"
#include "util/thread_map.h"
#include "util/evlist.h"
#include "util/evsel.h"
#include "util/header.h"
#include "util/session.h"

#include "util/parse-options.h"
#include "util/parse-events.h"

#include "util/thread.h"
#include "util/sort.h"
#include "util/hist.h"

#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>

static int			default_interval;
static int			freq = 1000; /* 1 KHz */
static bool			inherit = false;
static bool			group = false;
static unsigned int		mmap_pages = 128;
static struct perf_evlist	*top_evlist;
static struct winsize		winsize;

/* Tag samples to be skipped. */
static const char *skip_symbols[] = {
	"default_idle",
	"native_safe_halt",
	"cpu_idle",
	"enter_idle",
	"exit_idle",
	"mwait_idle",
	"mwait_idle_with_hints",
	"poll_idle",
	"ppc64_runlatch_off",
	"pseries_dedicated_idle_sleep",
	NULL
};

static int symbol_filter(struct map *map __used, struct symbol *sym)
{
	const char *name = sym->name;
	int i;

	/*
	 * ppc64 uses function descriptors and appends a '.' to the
	 * start of every instruction address. Remove it.
	 */
	if (name[0] == '.')
		name++;

	if (!strcmp(name, "_text") ||
	    !strcmp(name, "_etext") ||
	    !strcmp(name, "_sinittext") ||
	    !strncmp("init_module", name, 11) ||
	    !strncmp("cleanup_module", name, 14) ||
	    strstr(name, "_text_start") ||
	    strstr(name, "_text_end"))
		return 1;

	for (i = 0; skip_symbols[i]; i++) {
		if (!strcmp(skip_symbols[i], name)) {
			sym->ignore = true;
			break;
		}
	}

	return 0;
}

static void sig_winch_handler(int sig __used)
{
	get_term_dimensions(&winsize);
}

static int perf_session__add_hist_entry(struct perf_session *session,
					struct addr_location *al,
					struct perf_sample *sample,
					struct perf_evsel *evsel)
{
	struct hist_entry *he;

	he = __hists__add_entry(&evsel->hists, al, NULL, sample->period);
	if (he == NULL)
		return -ENOMEM;

	evsel->hists.stats.total_period += sample->period;
	session->hists.stats.total_period += sample->period;
	hists__inc_nr_events(&evsel->hists, PERF_RECORD_SAMPLE);
	return 0;
}


static int perf_event__process_sample(union perf_event *event,
				      struct perf_sample *sample,
				      struct perf_evsel *evsel,
				      struct perf_session *session)
{
	struct addr_location al;

	if (perf_event__preprocess_sample(event, session, &al,
					  sample, symbol_filter) < 0) {
		pr_err("Problem processing sample event, skipping it.\n");
		return -1;
	}

	if (al.sym && al.sym->ignore)
		return 0;

	if (perf_session__add_hist_entry(session, &al, sample, evsel)) {
		pr_err("Problem incrementing symbol period, skipping event\n");
		return -1;
	}

	return 0;
}

static void perf_evlist__mmap_process_events_idx(struct perf_evlist *evlist,
						 int idx,
						 struct perf_session *session)
{
	struct perf_sample sample;
	union perf_event *event;
	int ret;

	while ((event = perf_evlist__mmap_read(evlist, idx)) != NULL) {
		ret = perf_session__parse_sample(session, event, &sample);
		if (ret) {
			pr_err("Can't parse sample, err = %d\n", ret);
			continue;
		}

		if (event->header.type == PERF_RECORD_SAMPLE) {
			struct perf_evsel *evsel;
			evsel = perf_evlist__id2evsel(session->evlist, sample.id);
			if (evsel != NULL)
				perf_event__process_sample(event, &sample, evsel, session);
		} else
			perf_event__process(event, &sample, session);
	}
}

static void perf_evlist__mmap_process_events(struct perf_evlist *evlist,
					     struct perf_session *session)
{
	int i;

	for (i = 0; i < evlist->nr_mmaps; i++)
		perf_evlist__mmap_process_events_idx(evlist, i, session);
}

static int perf_evlist__fprintf_hists(struct perf_evlist *evlist, FILE *fp)
{
	struct perf_evsel *evsel;

	list_for_each_entry(evsel, &evlist->entries, node) {
		const char *evname = event_name(evsel);

		hists__collapse_resort(&evsel->hists);
		hists__output_resort(&evsel->hists);
		fprintf(stdout, "%s: ", evname);
		fprintf(stdout, "sort__need_collapse %d: ", sort__need_collapse);
		fprintf(stdout, " samples: %d", evsel->hists.stats.nr_events[PERF_RECORD_SAMPLE]);
		fprintf(stdout, " hists->nr_entries: %" PRIu64 "\n", evsel->hists.nr_entries);
		hists__fprintf(&evsel->hists, NULL, false, false,
			       winsize.ws_row - 3, winsize.ws_col, fp);
	}

	return 0;
}

static void *display_thread(void *arg __used)
{
	static const char CONSOLE_CLEAR[] = "[H[2J";
	struct pollfd stdin_poll = { .fd = 0, .events = POLLIN };
	struct termios tc, save;
	int delay_msecs, c;

	tcgetattr(0, &save);
	tc = save;
	tc.c_lflag &= ~(ICANON | ECHO);
	tc.c_cc[VMIN] = 0;
	tc.c_cc[VTIME] = 0;

	do {
		delay_msecs = 2 * 1000;
		tcsetattr(0, TCSANOW, &tc);
		/* trash return*/
		getc(stdin);

		do {
			puts(CONSOLE_CLEAR);
			perf_evlist__fprintf_hists(top_evlist, stdout);
		} while (!poll(&stdin_poll, 1, delay_msecs) == 1);

		c = getc(stdin);
		tcsetattr(0, TCSAFLUSH, &save);
	} while (c != 'q');

	return NULL;
}

static int perf_evlist__start(struct perf_evlist *evlist)
{
	struct perf_evsel *evsel;

	list_for_each_entry(evsel, &evlist->entries, node) {
		struct perf_event_attr *attr = &evsel->attr;

		attr->sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_TID;

		if (freq) {
			attr->sample_type |= PERF_SAMPLE_PERIOD;
			attr->freq	  = 1;
			attr->sample_freq = freq;
		}

		if (evlist->nr_entries > 1) {
			attr->sample_type |= PERF_SAMPLE_ID;
			attr->read_format |= PERF_FORMAT_ID;
		}

		attr->mmap = 1;
		attr->inherit = inherit;
try_again:
		if (perf_evsel__open(evsel, evlist->cpus,
				     evlist->threads, group) < 0) {
			int err = errno;

			if (err == EPERM || err == EACCES) {
				ui__warning_paranoid();
				goto out_err;
			}
			/*
			 * If it's cycles then fall back to hrtimer
			 * based cpu-clock-tick sw evsel, which
			 * is always available even if no PMU support:
			 */
			if (attr->type == PERF_TYPE_HARDWARE &&
			    attr->config == PERF_COUNT_HW_CPU_CYCLES) {
				if (verbose)
					ui__warning("Cycles event not supported,\n"
						    "trying to fall back to cpu-clock-ticks\n");

				attr->type = PERF_TYPE_SOFTWARE;
				attr->config = PERF_COUNT_SW_CPU_CLOCK;
				goto try_again;
			}

			if (err == ENOENT) {
				ui__warning("The %s event is not supported.\n",
					    event_name(evsel));
				goto out_err;
			}

			ui__warning("The sys_perf_event_open() syscall "
				    "returned with %d (%s).  /bin/dmesg "
				    "may provide additional information.\n"
				    "No CONFIG_PERF_EVENTS=y kernel support "
				    "configured?\n", err, strerror(err));
			goto out_err;
		}
	}

	if (perf_evlist__mmap(evlist, mmap_pages, false) < 0) {
		ui__warning("Failed to mmap with %d (%s)\n",
			    errno, strerror(errno));
		goto out_err;
	}

	return 0;
out_err:
	/* FIXME: stop all counters */
	return -1;
}

static int __cmd_top(void)
{
	pthread_t thread;
	struct perf_session *session;

	session = perf_session__new(NULL, O_WRONLY, false, false, NULL);

	if (session == NULL)
		return -ENOMEM;

	get_term_dimensions(&winsize);
	signal(SIGWINCH, sig_winch_handler);

	perf_evlist__start(top_evlist);
	session->evlist = top_evlist;
	perf_session__update_sample_type(session);

	perf_event__synthesize_threads(perf_event__process, session);

	/* Wait for a minimal set of events before starting the snapshot */
	poll(top_evlist->pollfd, top_evlist->nr_fds, 100);

	perf_evlist__mmap_process_events(top_evlist, session);

	if (pthread_create(&thread, NULL, display_thread, NULL)) {
		pr_err("Could not create display thread.\n");
		return -errno;
	}

	while (1) {
		u64 hits = session->hists.stats.total_period;

		perf_evlist__mmap_process_events(top_evlist, session);

		if (hits == session->hists.stats.total_period &&
		    poll(top_evlist->pollfd, top_evlist->nr_fds, 100) < 0)
			break;
	}

	return 0;
}

static const char * const top_usage[] = {
	"perf htop [<options>] <command>",
	NULL
};

static const struct option options[] = {
	OPT_INTEGER('c', "count", &default_interval, "event period to sample"),
	OPT_CALLBACK('e', "event", &top_evlist, "event",
		     "event selector. use 'perf list' to list available events",
		     parse_events_option),
	OPT_INTEGER('F', "freq", &freq, "profile at this frequency"),
	OPT_BOOLEAN('g', "group", &group,
		    "put the counters into a counter group"),
	OPT_BOOLEAN('i', "inherit", &inherit,
		    "child tasks inherit counters"),
	OPT_UINTEGER('m', "mmap-pages", &mmap_pages, "number of mmap data pages"),
	OPT_STRING('s', "sort", &sort_order, "key[,key2...]",
		   "sort by key(s): pid, comm, dso, symbol, parent"),
	OPT_INCR('v', "verbose", &verbose,
		 "be more verbose (show symbol address, etc)"),
	OPT_END()
};

int cmd_htop(int argc, const char **argv, const char *prefix __used)
{
	struct perf_evsel *pos;

	top_evlist = perf_evlist__new(NULL, NULL);
	if (top_evlist == NULL)
		return -ENOMEM;

	argc = parse_options(argc, argv, options, top_usage, 0);

	symbol_conf.exclude_other = false;

	setup_sorting(top_usage, options);

	/*
	 * Any (unrecognized) arguments left?
	 */
	if (argc)
		usage_with_options(top_usage, options);

	use_browser = 0;

	if (perf_evlist__create_maps(top_evlist, -1, -1, NULL) < 0)
		usage_with_options(top_usage, options);

	if (top_evlist->nr_entries == 0 &&
	    perf_evlist__add_default(top_evlist) < 0) {
		pr_err("Not enough memory for event selector list\n");
		return -ENOMEM;
	}

	/*
	 * User specified count overrides default frequency.
	 */
	if (default_interval)
		freq = 0;
	else if (freq) {
		default_interval = freq;
	} else {
		pr_err("frequency and count are zero, aborting\n");
		return -EINVAL;
	}

	list_for_each_entry(pos, &top_evlist->entries, node) {
		if (perf_evsel__alloc_fd(pos, top_evlist->cpus->nr,
					 top_evlist->threads->nr) < 0)
			goto out_free_fd;
		/*
		 * Fill in the ones not specifically initialized via -c:
		 */
		if (pos->attr.sample_period)
			continue;

		pos->attr.sample_period = default_interval;
	}

	if (perf_evlist__alloc_pollfd(top_evlist) < 0 ||
	    perf_evlist__alloc_mmap(top_evlist) < 0)
		goto out_free_fd;

	if (symbol__init() < 0)
		return -ENOMEM;

	return __cmd_top();
out_free_fd:
	perf_evlist__delete(top_evlist);
	return -ENOMEM;
}
