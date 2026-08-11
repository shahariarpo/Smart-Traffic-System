/* ============================================================================
 * main.c
 *
 * Command-line interface for the Smart Traffic Signal & Ambulance
 * Priority System. Ties together graph.c (network storage), dijkstra.c
 * (shortest-path routing, normal and emergency), and signal.c (green-
 * light timing) behind a menu-driven CLI.
 *
 * INPUT VALIDATION PHILOSOPHY
 * ----------------------------
 * Every read from stdin goes through read_line() (bounded fgets, never
 * gets/scanf-into-a-raw-buffer) and every numeric conversion goes
 * through parse_int()/parse_double() (strtol/strtod with full error
 * checking: rejects empty input, trailing garbage, and out-of-range
 * values) rather than raw scanf("%d", ...), which is notoriously unsafe
 * for interactive CLIs -- a non-numeric line fed to a bare "%d" leaves
 * the offending characters in the input buffer and desyncs every
 * subsequent read, and scanf gives no way to distinguish "0" from "not
 * a number" from a genuinely failed read. This module is where every
 * one of the spec's "invalid input" edge cases (negative weights typed
 * by the user, missing/garbage node ids, empty input) is actually
 * caught and turned into a friendly re-prompt rather than a crash.
 * ==========================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>

#include "graph.h"
#include "dijkstra.h"
#include "signal.h"

#define MAX_CITY_INTERSECTIONS 256
#define LINE_BUF_SIZE 256

/* ------------------------------------------------------------------
 * Low-level, safe input helpers
 * ------------------------------------------------------------------ */

/* Reads one line from stdin into buf (capacity bytes), stripping the
 * trailing newline if present. Returns 1 on success, 0 on EOF/error
 * (e.g. stdin closed / piped input exhausted) -- callers must check
 * this return value rather than assuming a line was always read, so
 * that piped/redirected input running out does not spin the menu loop
 * forever or read uninitialized memory. Also drains the rest of an
 * over-long line from the stream so a too-long line doesn't leak
 * leftover characters into the next read. */
static int read_line(char *buf, size_t capacity) {
    if (fgets(buf, (int)capacity, stdin) == NULL) {
        return 0;
    }
    size_t len = strlen(buf);
    if (len > 0 && buf[len - 1] == '\n') {
        buf[len - 1] = '\0';
    } else if (len == capacity - 1) {
        /* Line was longer than the buffer and had no newline yet in it --
         * drain the remainder of the actual input line so it doesn't
         * bleed into the next read_line() call. */
        int c;
        while ((c = getchar()) != '\n' && c != EOF) {
            /* discard */
        }
    }
    return 1;
}

/* Returns 1 if every character in s is whitespace (or s is empty), 0
 * otherwise. Used to reject blank-line input distinctly from genuinely
 * bad input, so we can give a clearer "input was empty" message. */
static int is_blank(const char *s) {
    for (const char *p = s; *p != '\0'; p++) {
        if (!isspace((unsigned char)*p)) {
            return 0;
        }
    }
    return 1;
}

/* Strict integer parse: accepts optional leading/trailing whitespace,
 * an optional leading '-' or '+', and digits -- nothing else. Rejects
 * empty input, pure whitespace, trailing garbage after the number
 * (e.g. "12abc"), and values outside int range. Returns 1 and writes
 * *out on success; returns 0 on any failure and leaves *out untouched. */
static int parse_int(const char *s, int *out) {
    if (s == NULL || is_blank(s)) {
        return 0;
    }
    while (isspace((unsigned char)*s)) s++;

    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (end == s) {
        return 0; /* no digits consumed at all */
    }
    if (errno == ERANGE) {
        return 0; /* value overflowed `long` itself (e.g. a 40-digit number) */
    }
    while (isspace((unsigned char)*end)) end++;
    if (*end != '\0') {
        return 0; /* trailing non-numeric garbage */
    }
    if (v < INT_MIN || v > INT_MAX) {
        return 0; /* fit in `long` but not in `int` -- still reject */
    }
    *out = (int)v;
    return 1;
}

/* Strict double parse: same philosophy as parse_int but for weights /
 * travel times, which are naturally fractional (e.g. 4.5 minutes). */
static int parse_double(const char *s, double *out) {
    if (s == NULL || is_blank(s)) {
        return 0;
    }
    while (isspace((unsigned char)*s)) s++;

    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) {
        return 0;
    }
    while (isspace((unsigned char)*end)) end++;
    if (*end != '\0') {
        return 0;
    }
    *out = v;
    return 1;
}

/* Prompts, reads a line, and strict-parses it as an int, re-prompting
 * on any failure until a valid value is entered or stdin is exhausted.
 * Returns 1 on success (value in *out), 0 if stdin ran out (EOF) before
 * a valid value was entered -- callers must check this so an EOF during
 * a piped test run cleanly aborts the current action instead of looping
 * forever. */
static int prompt_int(const char *prompt, int *out) {
    char buf[LINE_BUF_SIZE];
    for (;;) {
        printf("%s", prompt);
        fflush(stdout);
        if (!read_line(buf, sizeof(buf))) {
            return 0;
        }
        if (parse_int(buf, out)) {
            return 1;
        }
        printf("  [!] Invalid integer. Please enter a whole number.\n");
    }
}

static int prompt_double(const char *prompt, double *out) {
    char buf[LINE_BUF_SIZE];
    for (;;) {
        printf("%s", prompt);
        fflush(stdout);
        if (!read_line(buf, sizeof(buf))) {
            return 0;
        }
        if (parse_double(buf, out)) {
            return 1;
        }
        printf("  [!] Invalid number. Please enter a numeric value (e.g. 4.5).\n");
    }
}

/* Prompts for a non-blank line of text (e.g. an intersection name).
 * Re-prompts on a blank/empty line. Returns 1 on success, 0 on EOF. */
static int prompt_line(const char *prompt, char *out, size_t capacity) {
    for (;;) {
        printf("%s", prompt);
        fflush(stdout);
        if (!read_line(out, capacity)) {
            return 0;
        }
        if (!is_blank(out)) {
            return 1;
        }
        printf("  [!] Input cannot be empty.\n");
    }
}

/* ------------------------------------------------------------------
 * Vertex-selection helpers: let the user identify an intersection by
 * typed name (case-sensitive exact match, matching graph_find_vertex_by_name)
 * rather than by memorizing numeric ids, while still supporting a
 * numeric id directly for convenience. Centralizing this here means
 * every menu action gets the same "missing node" handling for free.
 * ------------------------------------------------------------------ */

/* Resolves user text to a vertex id: tries an exact name match first,
 * and if that fails, tries interpreting the text as a numeric id.
 * Returns the id, or GRAPH_INVALID_VERTEX if neither resolves -- this
 * is the single place that implements the spec's "missing nodes"
 * input-validation edge case for every menu action that needs to look
 * up an intersection. */
static int resolve_vertex(const Graph *graph, const char *text) {
    int id = graph_find_vertex_by_name(graph, text);
    if (id != GRAPH_INVALID_VERTEX) {
        return id;
    }
    int as_num;
    if (parse_int(text, &as_num) && graph_vertex_exists(graph, as_num)) {
        return as_num;
    }
    return GRAPH_INVALID_VERTEX;
}

static int prompt_vertex(const Graph *graph, const char *prompt) {
    char buf[LINE_BUF_SIZE];
    if (!prompt_line(prompt, buf, sizeof(buf))) {
        return GRAPH_INVALID_VERTEX;
    }
    int id = resolve_vertex(graph, buf);
    if (id == GRAPH_INVALID_VERTEX) {
        printf("  [!] No intersection matches \"%s\" (checked both name and id).\n", buf);
    }
    return id;
}

/* ------------------------------------------------------------------
 * Menu actions
 * ------------------------------------------------------------------ */

/* Returns 1 if every character in s is a decimal digit and s is
 * non-empty (i.e. s parses as a plain non-negative integer with no
 * sign or other characters). Used to reject purely-numeric intersection
 * names, since resolve_vertex() tries a name match before an id match:
 * naming a vertex "1" would make that name permanently shadow vertex id
 * 1 in every later "name or id" lookup, so we close off the ambiguity
 * at creation time instead of leaving it as a latent lookup bug. */
static int is_all_digits(const char *s) {
    if (s == NULL || s[0] == '\0') {
        return 0;
    }
    for (const char *p = s; *p != '\0'; p++) {
        if (!isdigit((unsigned char)*p)) {
            return 0;
        }
    }
    return 1;
}

static void action_add_intersection(Graph *graph) {
    char name[GRAPH_MAX_NAME_LEN];
    for (;;) {
        if (!prompt_line("  Intersection name: ", name, sizeof(name))) {
            return;
        }
        if (is_all_digits(name)) {
            printf("  [!] A purely numeric name (\"%s\") is not allowed -- it would be "
                   "indistinguishable from an intersection id in later lookups. "
                   "Please include at least one letter.\n", name);
            continue;
        }
        break;
    }
    int id;
    GraphStatus status = graph_add_vertex(graph, name, &id);
    switch (status) {
        case GRAPH_OK:
            printf("  [OK] Added intersection \"%s\" with id %d.\n", name, id);
            break;
        case GRAPH_ERR_DUPLICATE_VERTEX:
            printf("  [!] An intersection named \"%s\" already exists.\n", name);
            break;
        case GRAPH_ERR_CAPACITY_EXCEEDED:
            printf("  [!] City is at capacity (%d intersections). Cannot add more.\n",
                   graph->capacity);
            break;
        default:
            printf("  [!] Could not add intersection (error code %d).\n", (int)status);
            break;
    }
}

static void action_add_road(Graph *graph) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet. Add at least two intersections first.\n");
        return;
    }

    int src = prompt_vertex(graph, "  From intersection (name or id): ");
    if (src == GRAPH_INVALID_VERTEX) return;
    int dest = prompt_vertex(graph, "  To intersection (name or id): ");
    if (dest == GRAPH_INVALID_VERTEX) return;

    double weight;
    if (!prompt_double("  Travel time / distance weight (must be > 0): ", &weight)) return;

    char dirbuf[LINE_BUF_SIZE];
    if (!prompt_line("  Bidirectional road? (y/n): ", dirbuf, sizeof(dirbuf))) return;
    int bidir = (dirbuf[0] == 'y' || dirbuf[0] == 'Y');

    GraphStatus status = graph_add_edge(graph, src, dest, weight, bidir);
    switch (status) {
        case GRAPH_OK:
            printf("  [OK] Road added: %s -> %s (weight %.2f)%s\n",
                   graph_vertex_name(graph, src), graph_vertex_name(graph, dest),
                   weight, bidir ? " [bidirectional]" : " [one-way]");
            break;
        case GRAPH_ERR_SELF_LOOP:
            printf("  [!] A road cannot start and end at the same intersection.\n");
            break;
        case GRAPH_ERR_NEGATIVE_WEIGHT:
            printf("  [!] Travel time must be strictly positive (got %.2f). "
                   "Negative or zero weights are not valid for shortest-path routing.\n",
                   weight);
            break;
        case GRAPH_ERR_DUPLICATE_EDGE:
            printf("  [!] A road from %s to %s already exists. "
                   "(To change it, this build has no in-place edit; block/unblock is supported.)\n",
                   graph_vertex_name(graph, src), graph_vertex_name(graph, dest));
            break;
        case GRAPH_ERR_ALLOC_FAILED:
            printf("  [!] Memory allocation failed while adding the road.\n");
            break;
        default:
            printf("  [!] Could not add road (error code %d).\n", (int)status);
            break;
    }
}

static void action_block_or_unblock(Graph *graph, int block) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet.\n");
        return;
    }
    int src = prompt_vertex(graph, block ? "  Block road FROM intersection (name or id): "
                                          : "  Unblock road FROM intersection (name or id): ");
    if (src == GRAPH_INVALID_VERTEX) return;
    int dest = prompt_vertex(graph, "  ...TO intersection (name or id): ");
    if (dest == GRAPH_INVALID_VERTEX) return;

    GraphStatus status = block ? graph_block_edge(graph, src, dest)
                                : graph_unblock_edge(graph, src, dest);
    if (status == GRAPH_OK) {
        printf("  [OK] Road %s -> %s is now %s.\n",
               graph_vertex_name(graph, src), graph_vertex_name(graph, dest),
               block ? "BLOCKED (accident/closure simulated)" : "OPEN again");
    } else if (status == GRAPH_ERR_EDGE_NOT_FOUND) {
        printf("  [!] No road exists from %s to %s.\n",
               graph_vertex_name(graph, src), graph_vertex_name(graph, dest));
    } else {
        printf("  [!] Operation failed (error code %d).\n", (int)status);
    }
}

static void action_set_density(Graph *graph) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet.\n");
        return;
    }
    int v = prompt_vertex(graph, "  Intersection (name or id): ");
    if (v == GRAPH_INVALID_VERTEX) return;
    int density;
    if (!prompt_int("  Current vehicle count/density (>= 0): ", &density)) return;

    GraphStatus status = graph_set_density(graph, v, density);
    if (status == GRAPH_OK) {
        int green = signal_compute_green_time(graph, v);
        printf("  [OK] %s density set to %d. Recommended green-light time: %d seconds.\n",
               graph_vertex_name(graph, v), density, green);
    } else if (status == GRAPH_ERR_NEGATIVE_WEIGHT) {
        printf("  [!] Density cannot be negative (got %d).\n", density);
    } else {
        printf("  [!] Could not set density (error code %d).\n", (int)status);
    }
}

/* Shared core for "find route": runs Dijkstra in the given mode from
 * src to dest, prints a full human-readable report (step-by-step path,
 * total time, and active signal sequence), and returns the total travel
 * time via *out_total (GRAPH_INFINITY_TIME if unreachable) so that
 * comparison mode can call this twice and diff the results. Returns 1
 * if a path was found, 0 if the target is unreachable (not a crash-
 * worthy error -- printed as a clear message either way). */
static int run_and_report_route(const Graph *graph, int src, int dest,
                                  RouteMode mode, double *out_total) {
    DijkstraResult *result = dijkstra_run(graph, src, mode);
    if (result == NULL) {
        printf("  [!] Route calculation failed unexpectedly.\n");
        if (out_total) *out_total = GRAPH_INFINITY_TIME;
        return 0;
    }

    int *path = (int *)malloc((size_t)graph->num_vertices * sizeof(int));
    if (path == NULL) {
        printf("  [!] Memory allocation failed while reconstructing the route.\n");
        dijkstra_free_result(result);
        if (out_total) *out_total = GRAPH_INFINITY_TIME;
        return 0;
    }

    int path_len;
    int rc = dijkstra_reconstruct_path(result, dest, path, &path_len);

    if (rc == -1) {
        printf("  [!] UNREACHABLE: There is no valid route from \"%s\" to \"%s\"\n"
               "      given the current road network and blocked roads.\n",
               graph_vertex_name(graph, src), graph_vertex_name(graph, dest));
        if (out_total) *out_total = GRAPH_INFINITY_TIME;
        free(path);
        dijkstra_free_result(result);
        return 0;
    }

    printf("\n  === %s ROUTE: %s -> %s ===\n",
           mode == ROUTE_MODE_EMERGENCY ? "EMERGENCY (AMBULANCE)" : "NORMAL",
           graph_vertex_name(graph, src), graph_vertex_name(graph, dest));

    printf("\n  Step-by-step navigation:\n");
    for (int i = 0; i < path_len; i++) {
        printf("    %d. %s\n", i + 1, graph_vertex_name(graph, path[i]));
        if (i < path_len - 1) {
            /* Look up the edge weight actually used between consecutive
             * path vertices for a readable "-- Xmin -->" annotation.
             * This is a small O(deg) lookup per step purely for display;
             * it does not affect the already-computed shortest distance. */
            Edge *e = graph->vertices[path[i]].head;
            while (e != NULL && e->dest != path[i + 1]) e = e->next;
            if (e != NULL) {
                double shown_weight = e->weight;
                if (mode == ROUTE_MODE_EMERGENCY) {
                    shown_weight *= EMERGENCY_SPEED_FACTOR;
                }
                printf("         |\n         | %.2f min\n         v\n", shown_weight);
            }
        }
    }

    printf("\n  Total estimated travel time: %.2f minutes\n", result->dist[dest]);

    printf("\n  Active signal sequence along this route:\n");
    signal_print_sequence(graph, path, path_len, mode == ROUTE_MODE_EMERGENCY);

    if (out_total) *out_total = result->dist[dest];

    free(path);
    dijkstra_free_result(result);
    return 1;
}

static void action_normal_route(const Graph *graph) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet.\n");
        return;
    }
    int src = prompt_vertex(graph, "  Start intersection (name or id): ");
    if (src == GRAPH_INVALID_VERTEX) return;
    int dest = prompt_vertex(graph, "  Destination intersection (name or id): ");
    if (dest == GRAPH_INVALID_VERTEX) return;

    run_and_report_route(graph, src, dest, ROUTE_MODE_NORMAL, NULL);
}

static void action_emergency_route(const Graph *graph) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet.\n");
        return;
    }
    int src = prompt_vertex(graph, "  Ambulance current location (name or id): ");
    if (src == GRAPH_INVALID_VERTEX) return;
    int dest = prompt_vertex(graph, "  Emergency destination (name or id): ");
    if (dest == GRAPH_INVALID_VERTEX) return;

    run_and_report_route(graph, src, dest, ROUTE_MODE_EMERGENCY, NULL);
}

static void action_compare_routes(const Graph *graph) {
    if (graph->num_vertices == 0) {
        printf("  [!] No intersections exist yet.\n");
        return;
    }
    int src = prompt_vertex(graph, "  Start intersection (name or id): ");
    if (src == GRAPH_INVALID_VERTEX) return;
    int dest = prompt_vertex(graph, "  Destination intersection (name or id): ");
    if (dest == GRAPH_INVALID_VERTEX) return;

    double normal_total, emergency_total;
    printf("\n----------------------------------------------------------------\n");
    int normal_ok = run_and_report_route(graph, src, dest, ROUTE_MODE_NORMAL, &normal_total);
    printf("\n----------------------------------------------------------------\n");
    int emergency_ok = run_and_report_route(graph, src, dest, ROUTE_MODE_EMERGENCY, &emergency_total);
    printf("\n----------------------------------------------------------------\n");

    printf("\n  === COMPARISON SUMMARY ===\n");
    if (!normal_ok && !emergency_ok) {
        printf("  Neither a normal nor an emergency route exists between these "
               "intersections with the current road network.\n");
        return;
    }
    if (!normal_ok) {
        printf("  Normal route:     UNREACHABLE\n");
        printf("  Emergency route:  %.2f minutes\n", emergency_total);
        printf("  (No normal comparison possible -- only the emergency route is viable.)\n");
        return;
    }
    if (!emergency_ok) {
        /* Cannot actually happen given the current weight model (emergency
         * weights are a strict fraction of normal weights over the same
         * edge set, so emergency reachability is always a superset of
         * normal reachability) -- handled anyway so the comparison never
         * assumes a relationship between the two runs that isn't
         * independently re-verified. */
        printf("  Normal route:     %.2f minutes\n", normal_total);
        printf("  Emergency route:  UNREACHABLE\n");
        return;
    }

    double saved = normal_total - emergency_total;
    double pct = (normal_total > 0.0) ? (saved / normal_total) * 100.0 : 0.0;
    printf("  Normal route travel time:     %.2f minutes\n", normal_total);
    printf("  Emergency route travel time:  %.2f minutes\n", emergency_total);
    printf("  Time saved with priority:     %.2f minutes (%.1f%% faster)\n", saved, pct);
}

static void action_view_network(const Graph *graph) {
    printf("\n  === CURRENT ROAD NETWORK ===\n");
    graph_print(graph);
}

/* ------------------------------------------------------------------
 * Menu loop
 * ------------------------------------------------------------------ */

static void print_menu(void) {
    printf("\n"
           "==================================================================\n"
           " SMART TRAFFIC SIGNAL & AMBULANCE PRIORITY SYSTEM\n"
           "==================================================================\n"
           "  1. Add intersection\n"
           "  2. Add road (connect two intersections)\n"
           "  3. Set vehicle density at an intersection (normal traffic flow)\n"
           "  4. Block a road (simulate accident/closure)\n"
           "  5. Unblock a road (simulate clearance)\n"
           "  6. Find NORMAL route (shortest path)\n"
           "  7. Find EMERGENCY route (ambulance priority)\n"
           "  8. COMPARE normal vs emergency route\n"
           "  9. View current road network\n"
           "  0. Exit\n"
           "------------------------------------------------------------------\n"
           "  Choice: ");
}

int main(void) {
    Graph *graph = graph_create(MAX_CITY_INTERSECTIONS);
    if (graph == NULL) {
        fprintf(stderr, "FATAL: could not allocate the road network graph.\n");
        return EXIT_FAILURE;
    }

    printf("Smart Traffic Signal & Ambulance Priority System\n");
    printf("Pure C99/C11 -- Dijkstra's algorithm with a manual min-heap.\n");

    char buf[LINE_BUF_SIZE];
    int running = 1;
    while (running) {
        print_menu();
        fflush(stdout);
        if (!read_line(buf, sizeof(buf))) {
            printf("\n[EOF on input -- exiting.]\n");
            break;
        }

        int choice;
        if (!parse_int(buf, &choice)) {
            printf("  [!] Invalid menu choice. Please enter a number 0-9.\n");
            continue;
        }

        switch (choice) {
            case 1: action_add_intersection(graph); break;
            case 2: action_add_road(graph); break;
            case 3: action_set_density(graph); break;
            case 4: action_block_or_unblock(graph, 1); break;
            case 5: action_block_or_unblock(graph, 0); break;
            case 6: action_normal_route(graph); break;
            case 7: action_emergency_route(graph); break;
            case 8: action_compare_routes(graph); break;
            case 9: action_view_network(graph); break;
            case 0:
                printf("Exiting. Goodbye.\n");
                running = 0;
                break;
            default:
                printf("  [!] Unknown choice \"%d\". Please enter a number 0-9.\n", choice);
                break;
        }
    }

    /* Single, centralized cleanup point -- every malloc'd Edge and every
     * Vertex array is freed here via graph_destroy, regardless of which
     * menu path was last taken or how the loop was exited (normal '0'
     * choice or EOF on stdin). No other function in this file holds
     * long-lived heap allocations past its own return. */
    graph_destroy(graph);
    return EXIT_SUCCESS;
}
