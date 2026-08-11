/* ============================================================================
 * minheap.c
 *
 * Implementation of the array-backed binary min-heap declared in
 * minheap.h. See that header for the design rationale behind the
 * position[] side array.
 * ==========================================================================
 */

#include "minheap.h"
#include <stdlib.h>

static int parent_of(int i) { return (i - 1) / 2; }
static int left_of(int i)   { return 2 * i + 1; }
static int right_of(int i)  { return 2 * i + 2; }

/* Swaps the two heap nodes at indices i and j, AND keeps position[]
 * consistent for both affected vertices. Every place in this file that
 * moves a node inside `nodes[]` MUST go through this function -- a raw
 * swap that forgets to update position[] is exactly the bug that would
 * silently reintroduce O(V) decrease-key lookups (or worse, corrupt
 * heap state that a later decrease-key would read through a stale
 * index). */
static void heap_swap(MinHeap *heap, int i, int j) {
    HeapNode tmp = heap->nodes[i];
    heap->nodes[i] = heap->nodes[j];
    heap->nodes[j] = tmp;

    heap->position[heap->nodes[i].vertex] = i;
    heap->position[heap->nodes[j].vertex] = j;
}

/* Restores the min-heap property by moving the node at index i upward
 * as long as it is smaller than its parent. O(log V) -- at most the
 * height of the tree. */
static void sift_up(MinHeap *heap, int i) {
    while (i > 0 && heap->nodes[i].dist < heap->nodes[parent_of(i)].dist) {
        heap_swap(heap, i, parent_of(i));
        i = parent_of(i);
    }
}

/* Restores the min-heap property by moving the node at index i downward,
 * always swapping with the smaller of its two children, until it is
 * smaller than both or has no children. O(log V). */
static void sift_down(MinHeap *heap, int i) {
    for (;;) {
        int smallest = i;
        int l = left_of(i);
        int r = right_of(i);

        if (l < heap->count && heap->nodes[l].dist < heap->nodes[smallest].dist) {
            smallest = l;
        }
        if (r < heap->count && heap->nodes[r].dist < heap->nodes[smallest].dist) {
            smallest = r;
        }
        if (smallest == i) {
            break;
        }
        heap_swap(heap, i, smallest);
        i = smallest;
    }
}

MinHeap *heap_create(int capacity) {
    if (capacity <= 0) {
        return NULL;
    }

    MinHeap *heap = (MinHeap *)malloc(sizeof(MinHeap));
    if (heap == NULL) {
        return NULL;
    }

    heap->nodes = (HeapNode *)malloc((size_t)capacity * sizeof(HeapNode));
    if (heap->nodes == NULL) {
        free(heap);
        return NULL;
    }

    heap->position = (int *)malloc((size_t)capacity * sizeof(int));
    if (heap->position == NULL) {
        free(heap->nodes);
        free(heap);
        return NULL;
    }

    for (int i = 0; i < capacity; i++) {
        heap->position[i] = -1; /* -1 == "vertex i is not in the heap" */
    }

    heap->capacity = capacity;
    heap->count = 0;
    return heap;
}

void heap_destroy(MinHeap *heap) {
    if (heap == NULL) {
        return;
    }
    free(heap->nodes);
    free(heap->position);
    free(heap);
}

int heap_is_empty(const MinHeap *heap) {
    if (heap == NULL) {
        return 1;
    }
    return heap->count == 0;
}

int heap_push(MinHeap *heap, int v, double dist) {
    if (heap == NULL || v < 0 || v >= heap->capacity) {
        return -1;
    }
    if (heap->count >= heap->capacity) {
        return -1; /* should not happen in Dijkstra usage: each vertex is
                     pushed at most once, and capacity == vertex count */
    }

    int i = heap->count;
    heap->nodes[i].vertex = v;
    heap->nodes[i].dist = dist;
    heap->position[v] = i;
    heap->count++;

    sift_up(heap, i);
    return 0;
}

int heap_extract_min(MinHeap *heap, int *out_vertex, double *out_dist) {
    if (heap == NULL || out_vertex == NULL || out_dist == NULL) {
        return -1;
    }
    if (heap->count == 0) {
        return -1;
    }

    *out_vertex = heap->nodes[0].vertex;
    *out_dist = heap->nodes[0].dist;

    heap->position[heap->nodes[0].vertex] = -1; /* no longer in the heap */

    int last = heap->count - 1;
    heap->nodes[0] = heap->nodes[last];
    heap->position[heap->nodes[0].vertex] = 0;
    heap->count--;

    if (heap->count > 0) {
        sift_down(heap, 0);
    }
    return 0;
}

int heap_decrease_key(MinHeap *heap, int v, double new_dist) {
    if (heap == NULL || v < 0 || v >= heap->capacity) {
        return -1;
    }

    int idx = heap->position[v];
    if (idx == -1) {
        return 0; /* v is not in the heap (already finalized, or never
                    pushed) -- not an error, just nothing to do */
    }
    if (new_dist >= heap->nodes[idx].dist) {
        return 0; /* not actually a decrease -- no-op by contract */
    }

    heap->nodes[idx].dist = new_dist;
    sift_up(heap, idx);
    return 0;
}

int heap_contains(const MinHeap *heap, int v) {
    if (heap == NULL || v < 0 || v >= heap->capacity) {
        return 0;
    }
    return heap->position[v] != -1;
}
