/* ============================================================================
 * dijkstra.c
 *
 * Implementation of the heap-based Dijkstra declared in dijkstra.h.
 * ==========================================================================
 */

#include "dijkstra.h"
#include "minheap.h"
#include <stdlib.h>

DijkstraResult *dijkstra_run(const Graph *graph, int source, RouteMode mode) {
    if (graph == NULL) {
        return NULL;
    }
    if (graph->num_vertices == 0) {
        return NULL; /* empty graph: nothing to compute -- caller-visible
                        distinct failure from "ran but unreachable" */
    }
    if (!graph_vertex_exists(graph, source)) {
        return NULL;
    }

    int V = graph->num_vertices;

    DijkstraResult *result = (DijkstraResult *)malloc(sizeof(DijkstraResult));
    if (result == NULL) {
        return NULL;
    }
    result->dist = (double *)malloc((size_t)V * sizeof(double));
    result->pred = (int *)malloc((size_t)V * sizeof(int));
    if (result->dist == NULL || result->pred == NULL) {
        free(result->dist);
        free(result->pred);
        free(result);
        return NULL;
    }
    result->num_vertices = V;
    result->source = source;

    for (int i = 0; i < V; i++) {
        result->dist[i] = GRAPH_INFINITY_TIME;
        result->pred[i] = GRAPH_INVALID_VERTEX;
    }
    result->dist[source] = 0.0;

    MinHeap *heap = heap_create(V);
    if (heap == NULL) {
        free(result->dist);
        free(result->pred);
        free(result);
        return NULL;
    }

    /* Push every vertex once. Non-source vertices start at "infinity" --
     * pushing them now (rather than lazily on first discovery) keeps the
     * heap's capacity fixed at exactly V and avoids a second code path
     * for "insert if absent, else decrease-key," at the cost of O(V log V)
     * upfront pushes. This is standard practice and does not change the
     * overall O((V+E) log V) bound since V log V is already a term in it. */
    for (int i = 0; i < V; i++) {
        heap_push(heap, i, result->dist[i]);
    }

    /* visited[] marks vertices whose shortest distance is already final.
     * Needed because a vertex can still appear "in the heap" logically
     * at a stale distance for a brief window in some heap designs; here
     * with position[]-based decrease-key that specific staleness can't
     * happen, but we keep visited[] anyway as a second, explicit
     * safeguard against ever relaxing outward from a vertex twice --
     * cheap (O(V) space, O(1) per check) and it makes the algorithm's
     * core greedy invariant ("finalize once, relax once") visible in
     * the code rather than merely being an emergent property of the
     * heap's internals. */
    int *visited = (int *)calloc((size_t)V, sizeof(int));
    if (visited == NULL) {
        heap_destroy(heap);
        free(result->dist);
        free(result->pred);
        free(result);
        return NULL;
    }

    while (!heap_is_empty(heap)) {
        int u;
        double du;
        heap_extract_min(heap, &u, &du);

        if (visited[u]) {
            continue; /* defensive; see visited[] comment above */
        }
        visited[u] = 1;

        /* An extracted distance of infinity means every remaining vertex
         * in the heap is unreachable from source (Dijkstra processes
         * vertices in non-decreasing distance order, so once we hit
         * infinity, nothing smaller remains). Stopping early here is a
         * valid optimization, not required for correctness -- the loop
         * would terminate correctly on its own since no relaxation from
         * an infinite-distance vertex could ever improve anything -- but
         * it avoids O(V log V) of wasted extractions on a sparse graph
         * with many unreachable nodes. */
        if (du >= GRAPH_INFINITY_TIME) {
            break;
        }

        for (Edge *e = graph->vertices[u].head; e != NULL; e = e->next) {
            if (!e->active) {
                continue; /* blocked road: never usable, in either mode */
            }
            if (visited[e->dest]) {
                continue;
            }

            double w = e->weight;
            if (mode == ROUTE_MODE_EMERGENCY) {
                w = w * EMERGENCY_SPEED_FACTOR;
            }

            double candidate = result->dist[u] + w;
            if (candidate < result->dist[e->dest]) {
                result->dist[e->dest] = candidate;
                result->pred[e->dest] = u;
                heap_decrease_key(heap, e->dest, candidate);
            }
        }
    }

    free(visited);
    heap_destroy(heap);
    return result;
}

void dijkstra_free_result(DijkstraResult *result) {
    if (result == NULL) {
        return;
    }
    free(result->dist);
    free(result->pred);
    free(result);
}

int dijkstra_reconstruct_path(const DijkstraResult *result, int target,
                               int *out_path, int *out_len) {
    if (result == NULL || out_path == NULL || out_len == NULL) {
        return -2;
    }
    if (target < 0 || target >= result->num_vertices) {
        return -2;
    }

    if (result->dist[target] >= GRAPH_INFINITY_TIME) {
        *out_len = 0;
        return -1; /* unreachable target -- expected, not fatal */
    }

    /* Walk predecessor[] backwards from target to source, writing into a
     * temporary buffer, then reverse it into out_path. A path in a
     * shortest-path tree visits each vertex at most once, so it can
     * never contain more than V vertices -- out_path's caller-guaranteed
     * capacity of V is therefore always sufficient. */
    int *reversed = (int *)malloc((size_t)result->num_vertices * sizeof(int));
    if (reversed == NULL) {
        *out_len = 0;
        return -2;
    }

    int len = 0;
    int cur = target;
    while (cur != GRAPH_INVALID_VERTEX) {
        reversed[len++] = cur;
        if (cur == result->source) {
            break;
        }
        cur = result->pred[cur];
    }

    for (int i = 0; i < len; i++) {
        out_path[i] = reversed[len - 1 - i];
    }
    *out_len = len;

    free(reversed);
    return 0;
}
