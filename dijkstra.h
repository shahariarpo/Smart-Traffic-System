#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graph.h"

#define EMERGENCY_SPEED_FACTOR 0.6

typedef enum {
    ROUTE_MODE_NORMAL = 0,
    ROUTE_MODE_EMERGENCY = 1
} RouteMode;


typedef struct {
    double *dist;
    int *pred;
    int num_vertices;
    int source;
} DijkstraResult;


DijkstraResult *dijkstra_run(const Graph *graph, int source, RouteMode mode);


void dijkstra_free_result(DijkstraResult *result);

int dijkstra_reconstruct_path(const DijkstraResult *result, int target,
                               int *out_path, int *out_len);

#endif
