/* ============================================================================
 * signal.h
 *
 * Traffic signal green-light timing derived from vehicle density.
 *
 * MODEL
 * -----
 * Every intersection (vertex) carries a `density` value (set via
 * graph_set_density / the "simulate normal traffic flow" CLI action)
 * representing how many vehicles are currently queued at it. This module
 * converts that density into a recommended green-light duration.
 *
 * The timing rule is a simple, clearly-justified proportional model
 * rather than an opaque formula:
 *
 *     green_time = clamp(BASE_GREEN_SECONDS
 *                         + density * SECONDS_PER_VEHICLE,
 *                         MIN_GREEN_SECONDS, MAX_GREEN_SECONDS)
 *
 *   - BASE_GREEN_SECONDS: every signal gets at least this much green time
 *     even at zero density, so a signal never effectively vanishes.
 *   - SECONDS_PER_VEHICLE: each additional queued vehicle adds a fixed
 *     amount of green time, modeling the real-world fact that clearing
 *     more cars takes proportionally more time.
 *   - The result is clamped to [MIN_GREEN_SECONDS, MAX_GREEN_SECONDS] so
 *     that neither an empty intersection nor a gridlocked one produces
 *     an unusable signal plan (e.g. 0 seconds, or 20 minutes).
 *
 * This directly implements the spec's "Traffic signal timing adjustment
 * logic based on vehicle density/flow weights" requirement: it is a pure
 * function of density, O(1) per intersection, O(V) to compute a full
 * city-wide signal plan.
 *
 * SIGNAL SEQUENCE ALONG A ROUTE
 * ------------------------------
 * Given a path returned by dijkstra_reconstruct_path (an ordered list of
 * vertex ids), this module can produce the "active signal sequence" the
 * spec asks for in the output: for each intersection on the path (in
 * order), its computed green-light duration, plus (in emergency mode)
 * a note that the signal is preempted to green immediately rather than
 * waiting out a normal cycle -- modeling real-world ambulance signal
 * preemption systems.
 * ==========================================================================
 */

#ifndef SIGNAL_H
#define SIGNAL_H

#include "graph.h"

#define BASE_GREEN_SECONDS   15   /* floor: every signal gets this much     */
#define SECONDS_PER_VEHICLE   2    /* extra green time per queued vehicle    */
#define MIN_GREEN_SECONDS      10   /* absolute floor after clamping          */
#define MAX_GREEN_SECONDS       90   /* absolute ceiling after clamping        */

/* Computes the recommended green-light duration (seconds) for a single
 * intersection based on its current density, per the model documented
 * above. Returns -1 for an invalid graph/vertex id (a duration can never
 * legitimately be negative, so -1 is an unambiguous error sentinel here).
 * Time: O(1). */
int signal_compute_green_time(const Graph *graph, int vertex_id);

/* Prints the active signal sequence for an entire route (as produced by
 * dijkstra_reconstruct_path): for each intersection in `path`, its name
 * and computed green-light duration. If `emergency` is non-zero, also
 * notes that each signal is preempted (forced green immediately) for
 * the ambulance, which is the realistic behavior of emergency vehicle
 * preemption systems and visibly distinguishes the emergency output
 * from the normal one as the spec's comparison mode requires.
 * Time: O(path_len), i.e. O(V) worst case. */
void signal_print_sequence(const Graph *graph, const int *path, int path_len,
                            int emergency);

#endif /* SIGNAL_H */
