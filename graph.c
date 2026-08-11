/* ============================================================================
 * graph.c
 *
 * Implementation of the adjacency-list graph declared in graph.h.
 * See graph.h for the full design rationale; this file focuses on
 * per-function correctness and edge-case handling.
 * ==========================================================================
 */

#include "graph.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------
 * Internal helper: find the Edge node for src -> dest, or NULL.
 * Does NOT validate ids -- callers must do that first via
 * graph_vertex_exists(), since this is only ever called after that
 * check has already passed.
 * Time: O(deg(src))
 * ------------------------------------------------------------------ */
static Edge *find_edge(const Graph *graph, int src, int dest) {
    Edge *e = graph->vertices[src].head;
    while (e != NULL) {
        if (e->dest == dest) {
            return e;
        }
        e = e->next;
    }
    return NULL;
}

Graph *graph_create(int capacity) {
    if (capacity <= 0) {
        return NULL;
    }

    Graph *graph = (Graph *)malloc(sizeof(Graph));
    if (graph == NULL) {
        return NULL;
    }

    graph->vertices = (Vertex *)malloc((size_t)capacity * sizeof(Vertex));
    if (graph->vertices == NULL) {
        free(graph);
        return NULL;
    }

    graph->capacity = capacity;
    graph->num_vertices = 0;
    graph->num_edges = 0;

    /* Pre-initialize every slot so that even unused vertex slots are in a
     * defined state (head == NULL) rather than holding garbage, in case
     * any future code path ever iterates capacity instead of
     * num_vertices. Cheap at O(capacity) and done exactly once. */
    for (int i = 0; i < capacity; i++) {
        graph->vertices[i].id = i;
        graph->vertices[i].name[0] = '\0';
        graph->vertices[i].density = 0;
        graph->vertices[i].head = NULL;
    }

    return graph;
}

void graph_destroy(Graph *graph) {
    if (graph == NULL) {
        return;
    }

    /* Free every edge list. Each edge was individually malloc'd in
     * graph_add_edge, so each must be individually freed here -- freeing
     * only the vertex array would leak every Edge node. */
    if (graph->vertices != NULL) {
        for (int i = 0; i < graph->num_vertices; i++) {
            Edge *e = graph->vertices[i].head;
            while (e != NULL) {
                Edge *next = e->next;
                free(e);
                e = next;
            }
        }
        free(graph->vertices);
    }

    free(graph);
}

GraphStatus graph_add_vertex(Graph *graph, const char *name, int *out_id) {
    if (graph == NULL || name == NULL || out_id == NULL) {
        return GRAPH_ERR_NULL_POINTER;
    }
    if (name[0] == '\0') {
        return GRAPH_ERR_NULL_POINTER; /* treat empty name as invalid input */
    }
    if (graph->num_vertices >= graph->capacity) {
        return GRAPH_ERR_CAPACITY_EXCEEDED;
    }
    if (graph_find_vertex_by_name(graph, name) != GRAPH_INVALID_VERTEX) {
        return GRAPH_ERR_DUPLICATE_VERTEX;
    }

    int id = graph->num_vertices;
    /* strncpy + explicit null-termination: guards against a name argument
     * that is >= GRAPH_MAX_NAME_LEN characters, which strncpy alone would
     * leave unterminated. */
    strncpy(graph->vertices[id].name, name, GRAPH_MAX_NAME_LEN - 1);
    graph->vertices[id].name[GRAPH_MAX_NAME_LEN - 1] = '\0';
    graph->vertices[id].id = id;
    graph->vertices[id].density = 0;
    graph->vertices[id].head = NULL;

    graph->num_vertices++;
    *out_id = id;
    return GRAPH_OK;
}

GraphStatus graph_add_edge(Graph *graph, int src, int dest, double weight,
                            int bidirectional) {
    if (graph == NULL) {
        return GRAPH_ERR_NULL_POINTER;
    }
    if (!graph_vertex_exists(graph, src) || !graph_vertex_exists(graph, dest)) {
        return GRAPH_ERR_INVALID_VERTEX;
    }
    if (src == dest) {
        return GRAPH_ERR_SELF_LOOP;
    }
    /* Dijkstra's correctness proof relies on non-negative edge weights;
     * we go a step further and require strictly positive weights, since
     * a zero-weight road has no real-world travel-time meaning here and
     * strict positivity keeps the heap's decrease-key logic simple (no
     * need to special-case ties at distance 0 propagating instantly
     * through zero-weight cycles). This single check is what satisfies
     * the "handle negative weights" edge case from the spec. */
    if (weight <= 0.0) {
        return GRAPH_ERR_NEGATIVE_WEIGHT;
    }
    if (find_edge(graph, src, dest) != NULL) {
        return GRAPH_ERR_DUPLICATE_EDGE;
    }

    Edge *edge = (Edge *)malloc(sizeof(Edge));
    if (edge == NULL) {
        return GRAPH_ERR_ALLOC_FAILED;
    }
    edge->dest = dest;
    edge->weight = weight;
    edge->active = 1;
    edge->next = graph->vertices[src].head;
    graph->vertices[src].head = edge;
    graph->num_edges++;

    if (bidirectional) {
        /* Guard against the duplicate check firing on the reverse edge of
         * an already-existing road (e.g. user adds A->B then asks for a
         * bidirectional B->A that happens to coincide) -- if it already
         * exists, we still succeeded on the forward edge, so we do not
         * roll that back; we simply skip adding a second reverse edge. */
        if (find_edge(graph, dest, src) == NULL) {
            Edge *rev = (Edge *)malloc(sizeof(Edge));
            if (rev == NULL) {
                /* Partial failure: the forward edge stays (it is valid
                 * and already linked in), but we report the allocation
                 * failure so the caller knows the road is only one-way
                 * for now. This keeps the graph in a consistent,
                 * still-usable state rather than trying to unwind the
                 * first malloc. */
                return GRAPH_ERR_ALLOC_FAILED;
            }
            rev->dest = src;
            rev->weight = weight;
            rev->active = 1;
            rev->next = graph->vertices[dest].head;
            graph->vertices[dest].head = rev;
            graph->num_edges++;
        }
    }

    return GRAPH_OK;
}

GraphStatus graph_block_edge(Graph *graph, int src, int dest) {
    if (graph == NULL) {
        return GRAPH_ERR_NULL_POINTER;
    }
    if (!graph_vertex_exists(graph, src) || !graph_vertex_exists(graph, dest)) {
        return GRAPH_ERR_INVALID_VERTEX;
    }
    Edge *e = find_edge(graph, src, dest);
    if (e == NULL) {
        return GRAPH_ERR_EDGE_NOT_FOUND;
    }
    e->active = 0;
    return GRAPH_OK;
}

GraphStatus graph_unblock_edge(Graph *graph, int src, int dest) {
    if (graph == NULL) {
        return GRAPH_ERR_NULL_POINTER;
    }
    if (!graph_vertex_exists(graph, src) || !graph_vertex_exists(graph, dest)) {
        return GRAPH_ERR_INVALID_VERTEX;
    }
    Edge *e = find_edge(graph, src, dest);
    if (e == NULL) {
        return GRAPH_ERR_EDGE_NOT_FOUND;
    }
    e->active = 1;
    return GRAPH_OK;
}

GraphStatus graph_set_density(Graph *graph, int vertex_id, int density) {
    if (graph == NULL) {
        return GRAPH_ERR_NULL_POINTER;
    }
    if (!graph_vertex_exists(graph, vertex_id)) {
        return GRAPH_ERR_INVALID_VERTEX;
    }
    if (density < 0) {
        return GRAPH_ERR_NEGATIVE_WEIGHT; /* reuse: "negative not allowed" */
    }
    graph->vertices[vertex_id].density = density;
    return GRAPH_OK;
}

int graph_vertex_exists(const Graph *graph, int id) {
    if (graph == NULL) {
        return 0;
    }
    return (id >= 0 && id < graph->num_vertices);
}

int graph_find_vertex_by_name(const Graph *graph, const char *name) {
    if (graph == NULL || name == NULL) {
        return GRAPH_INVALID_VERTEX;
    }
    for (int i = 0; i < graph->num_vertices; i++) {
        if (strcmp(graph->vertices[i].name, name) == 0) {
            return i;
        }
    }
    return GRAPH_INVALID_VERTEX;
}

const char *graph_vertex_name(const Graph *graph, int id) {
    if (!graph_vertex_exists(graph, id)) {
        return NULL;
    }
    return graph->vertices[id].name;
}

void graph_print(const Graph *graph) {
    if (graph == NULL || graph->num_vertices == 0) {
        printf("  (graph is empty -- add intersections first)\n");
        return;
    }

    for (int i = 0; i < graph->num_vertices; i++) {
        const Vertex *v = &graph->vertices[i];
        printf("  [%d] %-20s (density: %d)\n", v->id, v->name, v->density);
        if (v->head == NULL) {
            printf("        (no outgoing roads)\n");
            continue;
        }
        for (Edge *e = v->head; e != NULL; e = e->next) {
            printf("        -> [%d] %-18s  time=%.2f  %s\n",
                   e->dest,
                   graph->vertices[e->dest].name,
                   e->weight,
                   e->active ? "OPEN" : "BLOCKED");
        }
    }
}
