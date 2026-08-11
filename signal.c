/* ============================================================================
 * signal.c
 *
 * Implementation of the density-based signal timing declared in signal.h.
 * ==========================================================================
 */

#include "signal.h"
#include <stdio.h>

/* Small local clamp helper -- kept private to this file since no other
 * module needs a generic clamp and pulling in an extra header for one
 * three-line function would be overkill. */
static int clamp_int(int value, int lo, int hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

int signal_compute_green_time(const Graph *graph, int vertex_id) {
    if (!graph_vertex_exists(graph, vertex_id)) {
        return -1;
    }

    int density = graph->vertices[vertex_id].density;
    /* density is validated non-negative at the point it is set
     * (graph_set_density rejects density < 0), so no further guard is
     * needed here -- but the clamp below still protects against an
     * absurdly high density producing an absurdly long green time. */
    int raw = BASE_GREEN_SECONDS + density * SECONDS_PER_VEHICLE;
    return clamp_int(raw, MIN_GREEN_SECONDS, MAX_GREEN_SECONDS);
}

void signal_print_sequence(const Graph *graph, const int *path, int path_len,
                            int emergency) {
    if (graph == NULL || path == NULL || path_len <= 0) {
        printf("  (no signal sequence to display)\n");
        return;
    }

    printf("  %-4s %-22s %-12s %s\n", "Seq", "Intersection", "Green(s)",
           emergency ? "Status" : "");
    printf("  ------------------------------------------------------------\n");

    for (int i = 0; i < path_len; i++) {
        int v = path[i];
        int green = signal_compute_green_time(graph, v);
        const char *name = graph_vertex_name(graph, v);

        if (emergency) {
            /* Emergency preemption: the signal is forced green
             * immediately for the ambulance rather than following its
             * normal timed cycle -- this is why the printed duration is
             * shown alongside a "PREEMPTED" status rather than replacing
             * it: the computed green_time still represents how long that
             * intersection would normally hold green (useful context /
             * what normal traffic experiences), while PREEMPTED
             * communicates that the ambulance itself does not wait for
             * it. */
            printf("  %-4d %-22s %-12d %s\n", i + 1, name, green,
                   "PREEMPTED (forced green)");
        } else {
            printf("  %-4d %-22s %-12d\n", i + 1, name, green);
        }
    }
}
