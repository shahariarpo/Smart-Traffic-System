/* ============================================================================
 * minheap.h
 *
 * A manual binary min-heap, implemented from scratch as an array, used as
 * the priority queue for Dijkstra's algorithm. This is the piece that
 * gets the overall algorithm from O(V^2) down to O((V + E) log V) --
 * without it, "find the unvisited vertex with the smallest tentative
 * distance" is a linear O(V) scan, and doing that V times costs O(V^2)
 * by itself before edges are even considered.
 *
 * Every heap node pairs a vertex id with its current tentative distance
 * (the min-heap property is maintained on distance).
 *
 * The key subtlety in a *from-scratch* Dijkstra heap is DECREASE-KEY:
 * when we relax an edge and find a shorter path to some vertex v that is
 * already sitting in the heap, we need to update v's distance and re-sift
 * it upward -- but a plain array-heap has no way to find "where is vertex
 * v right now" except a linear scan, which would silently degrade the
 * whole algorithm back to O(V^2).
 *
 * We solve this the standard way: a parallel `position[]` array indexed
 * by *vertex id* (not heap index) that always holds "the current index
 * of this vertex's node inside the heap array, or -1 if it is not
 * present." Every swap during sift-up/sift-down updates `position[]` for
 * both swapped vertices, so it never goes stale. This turns decrease-key
 * into an O(1) lookup + O(log V) sift-up.
 *
 * Complexity summary (V = number of vertices ever inserted):
 *   heap_create        O(V)        one-time array allocation
 *   heap_push          O(log V)
 *   heap_extract_min    O(log V)
 *   heap_decrease_key   O(log V)
 *   heap_is_empty       O(1)
 *   heap_destroy         O(1)  (single free; no per-node allocation)
 * Space: O(V) for the two parallel arrays (nodes[] and position[]).
 * ==========================================================================
 */

#ifndef MINHEAP_H
#define MINHEAP_H

#include <stddef.h>

typedef struct {
    int vertex;
    double dist;
} HeapNode;

typedef struct {
    HeapNode *nodes;   /* array-backed complete binary tree, size = count */
    int *position;      /* position[vertex_id] = index in nodes[], or -1  */
    int capacity;        /* max number of vertices this heap can ever hold */
    int count;             /* number of nodes currently in the heap        */
} MinHeap;

/* Allocates a heap that can hold up to `capacity` vertex entries (i.e.
 * capacity should equal the graph's vertex count). Returns NULL on
 * allocation failure or capacity <= 0.
 * Space: O(capacity). */
MinHeap *heap_create(int capacity);

/* Frees both internal arrays and the heap struct. Safe on NULL. */
void heap_destroy(MinHeap *heap);

/* Returns 1 if the heap currently holds zero nodes, else 0. Also returns
 * 1 (treats as empty) for a NULL heap, so callers can loop safely. */
int heap_is_empty(const MinHeap *heap);

/* Inserts vertex `v` with initial distance `dist`. Caller is responsible
 * for ensuring `v` is not already present (Dijkstra's setup loop inserts
 * each vertex exactly once, so this is guaranteed by construction there).
 * Returns 0 on success, -1 on NULL heap, capacity overflow, or invalid v.
 * Time: O(log V). */
int heap_push(MinHeap *heap, int v, double dist);

/* Removes and returns (via out params) the node with the smallest dist.
 * Returns 0 on success, -1 if the heap is empty or NULL, or if either
 * out pointer is NULL.
 * Time: O(log V). */
int heap_extract_min(MinHeap *heap, int *out_vertex, double *out_dist);

/* If vertex v is currently in the heap and new_dist < its current
 * distance, updates it and re-sifts upward to restore the heap
 * property. If v is not present, or new_dist is not smaller, this is a
 * safe no-op (returns 0 either way -- "no update needed" is not an
 * error). Returns -1 only for a NULL heap or an out-of-range vertex id.
 * Time: O(log V), thanks to position[] giving O(1) lookup of v's index. */
int heap_decrease_key(MinHeap *heap, int v, double new_dist);

/* Returns 1 if vertex v currently has a node in the heap, else 0.
 * O(1) via position[]. */
int heap_contains(const MinHeap *heap, int v);

#endif /* MINHEAP_H */
