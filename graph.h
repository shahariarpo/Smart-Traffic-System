/* ============================================================================
 * graph.h
 *
 * Graph Abstract Data Type for the Smart Traffic Signal & Ambulance
 * Priority System.
 *
 * Representation: Weighted, directed adjacency list built on an array of
 * linked lists. Roads are modeled as directed edges; the CLI layer adds a
 * road as two directed edges (A->B and B->A) when it represents a normal
 * bidirectional street. This lets us also support one-way streets by adding
 * a single directed edge, without needing a separate data structure.
 *
 * Why adjacency list over adjacency matrix:
 *   - Real road networks are sparse: E = O(V), not O(V^2). An intersection
 *     typically connects to 2-6 neighboring intersections regardless of how
 *     large the city grid grows.
 *   - Space:  Adjacency list  = O(V + E)
 *             Adjacency matrix = O(V^2)  <-- wasteful once V is large.
 *   - Dijkstra with a min-heap needs to enumerate "neighbors of node u"
 *     efficiently; adjacency list gives O(deg(u)) directly, whereas a
 *     matrix forces an O(V) scan per node regardless of actual degree.
 *
 * Every intersection (vertex) has:
 *   - an integer id (0..capacity-1), assigned at creation time
 *   - a human-readable name (e.g. "Main St & 5th Ave")
 *   - a vehicle density value, used by signal.c to compute green-light time
 *   - a linked list of outgoing roads (edges)
 *
 * Every road (edge) has:
 *   - a destination vertex id
 *   - a travel-time weight (double, must be > 0 for a normal, usable road)
 *   - an "active" flag -- a blocked/closed road is marked inactive rather
 *     than physically removed from the list. This models "set weight to
 *     infinity" from the spec while keeping removal O(1) and reversible
 *     (a road can be reopened later without re-adding it).
 *
 * All identifiers are validated against the graph's vertex count before
 * use; out-of-range ids are rejected by every public function rather than
 * causing undefined behavior.
 * ==========================================================================
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <stddef.h> /* size_t */

/* Sentinel used throughout the project to mean "no valid vertex" or
 * "unreachable". Chosen instead of -1 packed into an unsigned type to
 * avoid sign-conversion bugs; vertex ids are always non-negative ints. */
#define GRAPH_INVALID_VERTEX (-1)

/* Sentinel distance representing "infinite" travel time -- i.e. unreachable,
 * or a road that has been blocked. We do not use HUGE_VAL/INFINITY from
 * math.h to keep this header free of a math.h dependency and to guarantee
 * exact equality comparisons (IEEE infinities compare fine too, but a
 * plain large finite sentinel avoids any surprises from arithmetic like
 * INFINITY - INFINITY = NaN if a caller ever combines distances). */
#define GRAPH_INFINITY_TIME 1e18

/* Maximum length (including the null terminator) of an intersection name. */
#define GRAPH_MAX_NAME_LEN 64

/* --------------------------------------------------------------------
 * Return / status codes shared by graph.c, dijkstra.c and signal.c.
 * Every mutating or fallible operation returns one of these instead of
 * silently failing, so main.c can print a specific, honest error message
 * for every rejected input rather than crashing or guessing.
 * ------------------------------------------------------------------ */
typedef enum {
    GRAPH_OK = 0,
    GRAPH_ERR_NULL_POINTER,       /* a required pointer argument was NULL */
    GRAPH_ERR_INVALID_VERTEX,     /* vertex id out of range / not created */
    GRAPH_ERR_DUPLICATE_VERTEX,   /* name already used by another vertex */
    GRAPH_ERR_DUPLICATE_EDGE,     /* identical directed road already exists */
    GRAPH_ERR_SELF_LOOP,          /* source == destination */
    GRAPH_ERR_NEGATIVE_WEIGHT,    /* travel time must be > 0 */
    GRAPH_ERR_EDGE_NOT_FOUND,     /* attempted to block/unblock a missing road */
    GRAPH_ERR_CAPACITY_EXCEEDED,  /* graph already has max vertices */
    GRAPH_ERR_ALLOC_FAILED,       /* malloc/realloc returned NULL */
    GRAPH_ERR_EMPTY_GRAPH         /* operation requires at least one vertex */
} GraphStatus;

/* A single directed road out of some vertex. Stored as a singly linked
 * list node -- insertion at the head is O(1), which is all we need since
 * roads are typically added once at setup time (or rarely, at runtime)
 * and the list is otherwise only ever walked front-to-back. */
typedef struct Edge {
    int dest;              /* destination intersection id                */
    double weight;         /* travel time (minutes); always > 0 when active */
    int active;            /* 1 = usable, 0 = blocked (accident/closure)   */
    struct Edge *next;     /* next outgoing road from the same source      */
} Edge;

/* A single intersection. */
typedef struct {
    int id;
    char name[GRAPH_MAX_NAME_LEN];
    int density;            /* current vehicle density/count at this node  */
    Edge *head;              /* head of this vertex's outgoing-edge list    */
} Vertex;

/* The graph itself: a fixed-capacity array of vertices (capacity decided
 * at construction time from the caller's expected city size) plus a count
 * of how many are actually in use. Using a capacity-bounded array (rather
 * than a growable one) keeps vertex ids stable forever once assigned --
 * important because Dijkstra's distance/visited arrays are indexed
 * directly by vertex id, and a realloc-driven id remap would silently
 * invalidate any ids a caller is holding onto. */
typedef struct {
    Vertex *vertices;
    int capacity;   /* max number of intersections this graph can hold */
    int num_vertices; /* number of intersections actually in use       */
    int num_edges;    /* number of directed roads currently in the list */
} Graph;

/* ----------------------------- Lifecycle ----------------------------- */

/* Allocates a graph able to hold up to `capacity` intersections.
 * Returns NULL on allocation failure or if capacity <= 0.
 * Space complexity of the empty shell: O(capacity). */
Graph *graph_create(int capacity);

/* Frees every edge list, the vertex array, and the graph struct itself.
 * Safe to call with graph == NULL (no-op). After this call the pointer
 * must not be used again. */
void graph_destroy(Graph *graph);

/* ------------------------------ Mutators ------------------------------ */

/* Adds a new intersection with the given name. On success, writes the
 * newly assigned id to *out_id and returns GRAPH_OK.
 * Rejects: NULL graph/name/out_id, name that already exists (case-
 * sensitive exact match), or capacity already full. O(V) due to the
 * duplicate-name scan; fine at the scale of a lab project's road network. */
GraphStatus graph_add_vertex(Graph *graph, const char *name, int *out_id);

/* Adds a directed road src -> dest with the given weight (travel time).
 * If `bidirectional` is non-zero, also adds dest -> src with the same
 * weight in the same call (models a normal two-way street as a single
 * user-facing "add road" action while keeping the underlying storage
 * purely directed).
 * Rejects: invalid ids, src == dest (self-loop), weight <= 0 (covers both
 * the "negative weight" and "zero weight" edge cases -- Dijkstra requires
 * strictly positive weights to guarantee correctness), or an identical
 * directed edge that already exists (duplicate road). O(deg(src)) to
 * check for duplicates before inserting. */
GraphStatus graph_add_edge(Graph *graph, int src, int dest, double weight,
                            int bidirectional);

/* Marks the directed road src -> dest as blocked (accident/closure).
 * Does not remove it from memory -- it is simply skipped by Dijkstra's
 * neighbor relaxation step, and can be reopened later. O(deg(src)). */
GraphStatus graph_block_edge(Graph *graph, int src, int dest);

/* Reopens a previously blocked road. O(deg(src)). */
GraphStatus graph_unblock_edge(Graph *graph, int src, int dest);

/* Overwrites the vehicle density value stored at a vertex. Used by the
 * signal-timing module and by the "simulate normal traffic flow" CLI
 * action. O(1). */
GraphStatus graph_set_density(Graph *graph, int vertex_id, int density);

/* ------------------------------ Queries -------------------------------- */

/* Returns 1 if 0 <= id < graph->num_vertices, else 0. Also returns 0 for
 * a NULL graph. Every other function in this module calls this before
 * touching the vertex array, so out-of-range ids can never cause an
 * out-of-bounds access. */
int graph_vertex_exists(const Graph *graph, int id);

/* Looks up a vertex id by exact name match. Returns GRAPH_INVALID_VERTEX
 * if not found or on NULL input. O(V). */
int graph_find_vertex_by_name(const Graph *graph, const char *name);

/* Returns a read-only pointer to the vertex's name, or NULL if the id is
 * invalid. Callers must not modify or free the returned pointer. */
const char *graph_vertex_name(const Graph *graph, int id);

/* Prints a human-readable listing of every intersection and its outgoing
 * roads (including blocked ones, clearly marked) to stdout. Diagnostic /
 * CLI convenience function, not used internally by any algorithm. */
void graph_print(const Graph *graph);

#endif /* GRAPH_H */
