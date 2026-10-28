/*
 * ============================================================================
 *  SmartRoute - Intelligent Multi-Criteria City Route Planner
 *  Data Structures Mini Project (C, console based)
 * ----------------------------------------------------------------------------
 *  Modules (as in the project presentation)
 *   1. Network Manager        : graph with adjacency lists (linked lists)
 *   2. Route Engine           : BFS / DFS traversal, Dijkstra with a min-heap
 *   3. Preference Selector    : distance / time / cost / balanced edge weights
 *   4. Traffic & Closure Sim. : dynamic edge-weight updates, road closures
 *   5. Route Comparison       : table of routes + sorting by balanced score
 *   6. Explanation + Export   : "why this route" text, JSON export for a map
 *
 *  Scope: curated sample road network + simulated traffic.
 *         No live navigation or turn-by-turn directions is claimed.
 *
 *  Build :  gcc smartroute.c -o smartroute
 *  Run   :  ./smartroute        (Windows: smartroute.exe)
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <time.h>

#define MAX_NODES    50
#define NAME_LEN     32
#define COST_PER_KM  6.0          /* fuel/running cost in Rs per km        */
#define INF          DBL_MAX

/* ------------------------------------------------------------------------- */
/*  Data structures                                                          */
/* ------------------------------------------------------------------------- */

typedef enum { MODE_DISTANCE, MODE_TIME, MODE_COST, MODE_BALANCED } Mode;

/* One directed half of a road; every road is stored as two Edge nodes. */
typedef struct Edge {
    int to;
    double dist;        /* km                                    */
    double base_time;   /* minutes with no traffic               */
    double toll;        /* Rs                                    */
    double traffic;     /* multiplier on time (1.0 = free flow)  */
    int closed;         /* 1 = road unavailable                  */
    struct Edge *next;
} Edge;

typedef struct {
    int id;
    char name[NAME_LEN];
    double lat, lon;    /* used by the optional map layer        */
    Edge *head;         /* adjacency list                        */
} Node;

typedef struct {
    Node nodes[MAX_NODES];
    int n;
    double maxDist, maxTime, maxCost;   /* for normalisation (balanced mode) */
} Graph;

typedef struct { double wd, wt, wc; } Weights;   /* balanced weighting */

typedef struct {
    int found;
    char label[32];
    int path[MAX_NODES];
    int len;
    int roads;
    double dist, time, cost;
} Route;

/* Min-heap (priority queue) used by Dijkstra */
typedef struct { int node; double pri; } HeapItem;
typedef struct { HeapItem *a; int size, cap; } MinHeap;

static Route g_last_route;      /* remembered for JSON export */
static int   g_has_last = 0;

/* ------------------------------------------------------------------------- */
/*  Edge helpers                                                             */
/* ------------------------------------------------------------------------- */

static double edge_time(const Edge *e) { return e->base_time * e->traffic; }
static double edge_cost(const Edge *e) { return e->toll + e->dist * COST_PER_KM; }

static const char *mode_name(Mode m)
{
    switch (m) {
        case MODE_DISTANCE: return "Shortest distance";
        case MODE_TIME:     return "Fastest time";
        case MODE_COST:     return "Lowest cost";
        default:            return "Balanced";
    }
}

static double edge_weight(const Graph *g, const Edge *e, Mode m, const Weights *w)
{
    switch (m) {
        case MODE_DISTANCE: return e->dist;
        case MODE_TIME:     return edge_time(e);
        case MODE_COST:     return edge_cost(e);
        default:
            return w->wd * (e->dist       / g->maxDist)
                 + w->wt * (edge_time(e)  / g->maxTime)
                 + w->wc * (edge_cost(e)  / g->maxCost);
    }
}

/* ------------------------------------------------------------------------- */
/*  Module 1: Network Manager                                                */
/* ------------------------------------------------------------------------- */

static void graph_init(Graph *g)
{
    memset(g, 0, sizeof(*g));
    g->maxDist = g->maxTime = g->maxCost = 1.0;
}

static void graph_free(Graph *g)
{
    for (int i = 0; i < g->n; i++) {
        Edge *e = g->nodes[i].head;
        while (e) { Edge *t = e->next; free(e); e = t; }
        g->nodes[i].head = NULL;
    }
}

/* Recompute the maximum edge values (needed to normalise balanced mode). */
static void graph_update_norms(Graph *g)
{
    double md = 0, mt = 0, mc = 0;
    for (int i = 0; i < g->n; i++)
        for (Edge *e = g->nodes[i].head; e; e = e->next) {
            if (e->dist > md)        md = e->dist;
            if (edge_time(e) > mt)   mt = edge_time(e);
            if (edge_cost(e) > mc)   mc = edge_cost(e);
        }
    g->maxDist = md > 0 ? md : 1.0;
    g->maxTime = mt > 0 ? mt : 1.0;
    g->maxCost = mc > 0 ? mc : 1.0;
}

static int add_node(Graph *g, const char *name, double lat, double lon)
{
    if (g->n >= MAX_NODES) return -1;
    Node *nd = &g->nodes[g->n];
    nd->id = g->n;
    strncpy(nd->name, name, NAME_LEN - 1);
    nd->name[NAME_LEN - 1] = '\0';
    nd->lat = lat;
    nd->lon = lon;
    nd->head = NULL;
    return g->n++;
}

static Edge *find_edge(const Graph *g, int a, int b)
{
    for (Edge *e = g->nodes[a].head; e; e = e->next)
        if (e->to == b) return e;
    return NULL;
}

static Edge *new_edge(int to, double d, double t, double toll)
{
    Edge *e = (Edge *)malloc(sizeof(Edge));
    if (!e) { perror("malloc"); exit(1); }
    e->to = to; e->dist = d; e->base_time = t; e->toll = toll;
    e->traffic = 1.0; e->closed = 0; e->next = NULL;
    return e;
}

/* Adds a two-way road. Returns 1 on success, 0 on invalid/duplicate. */
static int add_road(Graph *g, int a, int b, double d, double t, double toll)
{
    if (a < 0 || b < 0 || a >= g->n || b >= g->n || a == b) return 0;
    if (find_edge(g, a, b)) return 0;
    Edge *e1 = new_edge(b, d, t, toll);
    e1->next = g->nodes[a].head; g->nodes[a].head = e1;
    Edge *e2 = new_edge(a, d, t, toll);
    e2->next = g->nodes[b].head; g->nodes[b].head = e2;
    graph_update_norms(g);
    return 1;
}

/* ------------------------------------------------------------------------- */
/*  Module 4: Traffic & Closure Simulator (dynamic graph updates)            */
/* ------------------------------------------------------------------------- */

static int set_traffic(Graph *g, int a, int b, double factor)
{
    Edge *e1 = find_edge(g, a, b), *e2 = find_edge(g, b, a);
    if (!e1 || !e2) return 0;
    e1->traffic = e2->traffic = factor;
    graph_update_norms(g);
    return 1;
}

static int set_closed(Graph *g, int a, int b, int closed)
{
    Edge *e1 = find_edge(g, a, b), *e2 = find_edge(g, b, a);
    if (!e1 || !e2) return 0;
    e1->closed = e2->closed = closed;
    return 1;
}

static void reset_conditions(Graph *g)
{
    for (int i = 0; i < g->n; i++)
        for (Edge *e = g->nodes[i].head; e; e = e->next) {
            e->traffic = 1.0;
            e->closed = 0;
        }
    graph_update_norms(g);
}

static void random_traffic(Graph *g)
{
    int changed = 0;
    for (int i = 0; i < g->n; i++)
        for (Edge *e = g->nodes[i].head; e; e = e->next)
            if (e->to > i && (rand() % 100) < 40) {
                double f = 1.2 + (rand() % 19) / 10.0;      /* 1.2 .. 3.0 */
                set_traffic(g, i, e->to, f);
                changed++;
            }
    printf("Random traffic applied to %d roads.\n", changed);
}

/* ------------------------------------------------------------------------- */
/*  Min-heap                                                                 */
/* ------------------------------------------------------------------------- */

static void heap_push(MinHeap *h, int node, double pri)
{
    if (h->size == h->cap) {
        h->cap = h->cap ? h->cap * 2 : 16;
        h->a = (HeapItem *)realloc(h->a, h->cap * sizeof(HeapItem));
        if (!h->a) { perror("realloc"); exit(1); }
    }
    int i = h->size++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h->a[p].pri <= pri) break;
        h->a[i] = h->a[p];
        i = p;
    }
    h->a[i].node = node;
    h->a[i].pri = pri;
}

static HeapItem heap_pop(MinHeap *h)
{
    HeapItem top = h->a[0];
    HeapItem last = h->a[--h->size];
    int i = 0;
    while (1) {
        int c = 2 * i + 1;
        if (c >= h->size) break;
        if (c + 1 < h->size && h->a[c + 1].pri < h->a[c].pri) c++;
        if (h->a[c].pri >= last.pri) break;
        h->a[i] = h->a[c];
        i = c;
    }
    if (h->size > 0) h->a[i] = last;
    return top;
}

/* ------------------------------------------------------------------------- */
/*  Module 2: Route Engine                                                   */
/* ------------------------------------------------------------------------- */

/* Rebuild the path from the predecessor array and total up its metrics. */
static void build_route(const Graph *g, const int prev[], int src, int dst,
                        const char *label, Route *r)
{
    memset(r, 0, sizeof(*r));
    strncpy(r->label, label, sizeof(r->label) - 1);

    int tmp[MAX_NODES], cnt = 0;
    for (int v = dst; v != -1 && cnt < MAX_NODES; v = prev[v]) tmp[cnt++] = v;
    if (cnt == 0 || tmp[cnt - 1] != src) { r->found = 0; return; }

    r->found = 1;
    r->len = cnt;
    for (int i = 0; i < cnt; i++) r->path[i] = tmp[cnt - 1 - i];
    r->roads = cnt - 1;

    for (int i = 0; i + 1 < r->len; i++) {
        Edge *e = find_edge(g, r->path[i], r->path[i + 1]);
        r->dist += e->dist;
        r->time += edge_time(e);
        r->cost += edge_cost(e);
    }
}

/* Dijkstra with a binary min-heap. Closed roads are skipped. */
static Route dijkstra_route(const Graph *g, int src, int dst, Mode m, const Weights *w)
{
    double d[MAX_NODES];
    int prev[MAX_NODES];
    for (int i = 0; i < g->n; i++) { d[i] = INF; prev[i] = -1; }

    MinHeap h = { NULL, 0, 0 };
    d[src] = 0;
    heap_push(&h, src, 0);

    while (h.size > 0) {
        HeapItem it = heap_pop(&h);
        int u = it.node;
        if (it.pri > d[u]) continue;            /* stale heap entry */
        if (u == dst) break;
        for (Edge *e = g->nodes[u].head; e; e = e->next) {
            if (e->closed) continue;
            double nd = d[u] + edge_weight(g, e, m, w);
            if (nd < d[e->to]) {
                d[e->to] = nd;
                prev[e->to] = u;
                heap_push(&h, e->to, nd);
            }
        }
    }
    free(h.a);

    Route r;
    if (d[dst] == INF) { memset(&r, 0, sizeof(r)); strcpy(r.label, mode_name(m)); }
    else build_route(g, prev, src, dst, mode_name(m), &r);
    return r;
}

/* BFS: route with the fewest road segments (unweighted). */
static Route bfs_route(const Graph *g, int src, int dst)
{
    int q[MAX_NODES], head = 0, tail = 0;
    int visited[MAX_NODES] = {0}, prev[MAX_NODES];
    for (int i = 0; i < g->n; i++) prev[i] = -1;

    q[tail++] = src; visited[src] = 1;
    while (head < tail) {
        int u = q[head++];
        if (u == dst) break;
        for (Edge *e = g->nodes[u].head; e; e = e->next)
            if (!e->closed && !visited[e->to]) {
                visited[e->to] = 1;
                prev[e->to] = u;
                q[tail++] = e->to;
            }
    }
    Route r;
    if (!visited[dst]) { memset(&r, 0, sizeof(r)); strcpy(r.label, "Fewest roads (BFS)"); }
    else build_route(g, prev, src, dst, "Fewest roads (BFS)", &r);
    return r;
}

static void bfs_order(const Graph *g, int start)
{
    int q[MAX_NODES], head = 0, tail = 0, visited[MAX_NODES] = {0};
    q[tail++] = start; visited[start] = 1;
    printf("BFS order from %s:\n  ", g->nodes[start].name);
    while (head < tail) {
        int u = q[head++];
        printf("%s%s", head > 1 ? " -> " : "", g->nodes[u].name);
        for (Edge *e = g->nodes[u].head; e; e = e->next)
            if (!e->closed && !visited[e->to]) { visited[e->to] = 1; q[tail++] = e->to; }
    }
    printf("\n  Reachable locations: %d of %d\n", tail, g->n);
}

static void dfs_visit(const Graph *g, int u, int visited[], int *count)
{
    visited[u] = 1;
    printf("%s%s", (*count)++ ? " -> " : "", g->nodes[u].name);
    for (Edge *e = g->nodes[u].head; e; e = e->next)
        if (!e->closed && !visited[e->to]) dfs_visit(g, e->to, visited, count);
}

static void dfs_order(const Graph *g, int start)
{
    int visited[MAX_NODES] = {0}, count = 0;
    printf("DFS order from %s:\n  ", g->nodes[start].name);
    dfs_visit(g, start, visited, &count);
    printf("\n  Reachable locations: %d of %d\n", count, g->n);
}

/* ------------------------------------------------------------------------- */
/*  Input helpers                                                            */
/* ------------------------------------------------------------------------- */

static void read_line(char *buf, int n)
{
    if (!fgets(buf, n, stdin)) { buf[0] = '\0'; return; }
    buf[strcspn(buf, "\r\n")] = '\0';
}

static int read_int(const char *prompt, int lo, int hi)
{
    char buf[64]; int v; char extra;
    while (1) {
        printf("%s", prompt);
        read_line(buf, sizeof(buf));
        if (sscanf(buf, "%d %c", &v, &extra) == 1 && v >= lo && v <= hi) return v;
        printf("  Please enter a whole number between %d and %d.\n", lo, hi);
    }
}

static double read_double(const char *prompt, double lo, double hi)
{
    char buf[64]; double v; char extra;
    while (1) {
        printf("%s", prompt);
        read_line(buf, sizeof(buf));
        if (sscanf(buf, "%lf %c", &v, &extra) == 1 && v >= lo && v <= hi) return v;
        printf("  Please enter a number between %.1f and %.1f.\n", lo, hi);
    }
}

/* ------------------------------------------------------------------------- */
/*  Output: lists, routes, comparison, explanation                           */
/* ------------------------------------------------------------------------- */

static void print_locations(const Graph *g)
{
    printf("\n  ID  Location\n  --  ------------------------\n");
    for (int i = 0; i < g->n; i++) printf("  %2d  %s\n", i, g->nodes[i].name);
}

static void print_roads(const Graph *g)
{
    printf("\n  %-18s %-18s %6s %7s %7s %6s %8s %s\n",
           "From", "To", "Km", "BaseMin", "Traffic", "EffMin", "Toll(Rs)", "Status");
    printf("  ------------------------------------------------------------------------------------\n");
    for (int i = 0; i < g->n; i++)
        for (Edge *e = g->nodes[i].head; e; e = e->next)
            if (e->to > i)
                printf("  %-18s %-18s %6.1f %7.1f   x%-4.1f %6.1f %8.0f %s\n",
                       g->nodes[i].name, g->nodes[e->to].name, e->dist, e->base_time,
                       e->traffic, edge_time(e), e->toll, e->closed ? "CLOSED" : "open");
}

static void print_path(const Graph *g, const Route *r)
{
    printf("  Path   : ");
    for (int i = 0; i < r->len; i++)
        printf("%s%s", g->nodes[r->path[i]].name, i + 1 < r->len ? " -> " : "\n");
}

static void print_route(const Graph *g, const Route *r)
{
    printf("\n[%s]\n", r->label);
    if (!r->found) { printf("  No route available (destination unreachable).\n"); return; }
    print_path(g, r);
    printf("  Roads  : %d\n  Distance: %.1f km\n  Time   : %.1f min\n  Cost   : Rs %.0f\n",
           r->roads, r->dist, r->time, r->cost);
}

static int same_path(const Route *a, const Route *b)
{
    if (!a->found || !b->found || a->len != b->len) return 0;
    for (int i = 0; i < a->len; i++) if (a->path[i] != b->path[i]) return 0;
    return 1;
}

/* Module 6: explain *why* a route was selected, relative to shortest distance. */
static void explain_route(const Graph *g, const Route *r, const Route *shortest,
                          Mode m, const Weights *w)
{
    (void)g;
    printf("\nWhy this route?\n");
    switch (m) {
        case MODE_DISTANCE: printf("  - It minimises total distance (%.1f km).\n", r->dist); break;
        case MODE_TIME:     printf("  - It minimises estimated travel time (%.1f min), including traffic.\n", r->time); break;
        case MODE_COST:     printf("  - It minimises estimated cost (Rs %.0f = tolls + Rs %.0f/km running cost).\n", r->cost, COST_PER_KM); break;
        default:            printf("  - It minimises a balanced score of normalised distance (%.0f%%), time (%.0f%%) and cost (%.0f%%).\n",
                                   w->wd * 100, w->wt * 100, w->wc * 100); break;
    }
    if (m == MODE_DISTANCE || same_path(r, shortest)) {
        printf("  - This is also the shortest-distance route, so there is no trade-off here.\n");
        return;
    }
    double dd = r->dist - shortest->dist;
    double dt = r->time - shortest->time;
    double dc = r->cost - shortest->cost;
    printf("  - Compared with the shortest-distance route it is %.1f km %s, %.1f min %s and Rs %.0f %s.\n",
           dd >= 0 ? dd : -dd, dd >= 0 ? "longer" : "shorter",
           dt >= 0 ? dt : -dt, dt >= 0 ? "slower" : "faster",
           dc >= 0 ? dc : -dc, dc >= 0 ? "costlier" : "cheaper");
}

/* Module 5: compare routes by every criterion; sorted by balanced score. */
typedef struct { Route r; double score; } Ranked;

static int cmp_ranked(const void *a, const void *b)
{
    double x = ((const Ranked *)a)->score, y = ((const Ranked *)b)->score;
    return (x > y) - (x < y);
}

static void compare_routes(const Graph *g, int src, int dst, const Weights *w)
{
    Ranked rr[5]; int n = 0;
    Mode modes[4] = { MODE_DISTANCE, MODE_TIME, MODE_COST, MODE_BALANCED };
    for (int i = 0; i < 4; i++) rr[n++].r = dijkstra_route(g, src, dst, modes[i], w);
    rr[n++].r = bfs_route(g, src, dst);

    if (!rr[0].r.found) {
        printf("\nNo route exists between %s and %s under current conditions.\n",
               g->nodes[src].name, g->nodes[dst].name);
        return;
    }

    double bd = INF, bt = INF, bc = INF;
    for (int i = 0; i < n; i++) {
        if (rr[i].r.dist < bd) bd = rr[i].r.dist;
        if (rr[i].r.time < bt) bt = rr[i].r.time;
        if (rr[i].r.cost < bc) bc = rr[i].r.cost;
    }
    /* balanced score of a whole route, relative to the best value per column */
    for (int i = 0; i < n; i++)
        rr[i].score = w->wd * rr[i].r.dist / bd + w->wt * rr[i].r.time / bt
                    + w->wc * rr[i].r.cost / (bc > 0 ? bc : 1.0);
    qsort(rr, n, sizeof(Ranked), cmp_ranked);

    printf("\nRoute comparison: %s -> %s   (sorted by balanced score, lower = better)\n",
           g->nodes[src].name, g->nodes[dst].name);
    printf("  %-4s %-22s %9s %9s %9s %6s\n", "Rank", "Strategy", "Km", "Min", "Rs", "Roads");
    printf("  -----------------------------------------------------------------\n");
    for (int i = 0; i < n; i++) {
        const Route *r = &rr[i].r;
        printf("  %-4d %-22s %8.1f%c %8.1f%c %8.0f%c %6d\n", i + 1, r->label,
               r->dist, r->dist <= bd + 1e-9 ? '*' : ' ',
               r->time, r->time <= bt + 1e-9 ? '*' : ' ',
               r->cost, r->cost <= bc + 1e-9 ? '*' : ' ', r->roads);
    }
    printf("  (* = best value in that column)\n\n");
    for (int i = 0; i < n; i++) {
        printf("  %-22s: ", rr[i].r.label);
        for (int k = 0; k < rr[i].r.len; k++)
            printf("%s%s", g->nodes[rr[i].r.path[k]].name, k + 1 < rr[i].r.len ? " > " : "\n");
    }
}

/* Export last route for the optional Leaflet/OpenStreetMap interface. */
static void export_json(const Graph *g, const Route *r, const char *file)
{
    FILE *f = fopen(file, "w");
    if (!f) { printf("Could not write %s\n", file); return; }
    fprintf(f, "{\n  \"strategy\": \"%s\",\n  \"distance_km\": %.2f,\n  \"time_min\": %.2f,\n"
               "  \"cost_rs\": %.2f,\n  \"path\": [\n", r->label, r->dist, r->time, r->cost);
    for (int i = 0; i < r->len; i++) {
        const Node *nd = &g->nodes[r->path[i]];
        fprintf(f, "    {\"id\": %d, \"name\": \"%s\", \"lat\": %.5f, \"lon\": %.5f}%s\n",
                nd->id, nd->name, nd->lat, nd->lon, i + 1 < r->len ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
    printf("Route exported to %s\n", file);
}

/* ------------------------------------------------------------------------- */
/*  Sample city network (curated, editable)                                  */
/* ------------------------------------------------------------------------- */

static void load_sample_city(Graph *g)
{
    add_node(g, "Central Station", 23.0225, 72.5714);   /* 0 */
    add_node(g, "City Mall",       23.0330, 72.5850);   /* 1 */
    add_node(g, "Airport",         23.0770, 72.6340);   /* 2 */
    add_node(g, "University",      23.0120, 72.5500);   /* 3 */
    add_node(g, "Tech Park",       23.0450, 72.5600);   /* 4 */
    add_node(g, "Old Market",      23.0180, 72.5900);   /* 5 */
    add_node(g, "Hospital",        23.0050, 72.5950);   /* 6 */
    add_node(g, "Stadium",         23.0600, 72.6000);   /* 7 */
    add_node(g, "Riverside",       23.0000, 72.6100);   /* 8 */
    add_node(g, "Harbor",          23.0400, 72.6500);   /* 9 */

    /*        a  b   km   min  toll */
    add_road(g, 0, 1,  3.0,  8.0,  0);
    add_road(g, 0, 5,  2.0,  6.0,  0);
    add_road(g, 0, 3,  6.0, 12.0,  0);
    add_road(g, 1, 2, 12.0, 14.0, 30);   /* expressway: fast but tolled */
    add_road(g, 1, 4,  5.0, 10.0,  0);
    add_road(g, 5, 6,  4.0, 10.0,  0);
    add_road(g, 5, 8,  6.0, 15.0,  0);
    add_road(g, 3, 4,  4.0,  9.0,  0);
    add_road(g, 3, 6,  5.0, 11.0,  0);
    add_road(g, 4, 7,  6.0,  8.0, 20);   /* flyover: tolled */
    add_road(g, 4, 2, 14.0, 20.0,  0);
    add_road(g, 2, 7,  7.0,  9.0, 25);
    add_road(g, 2, 9, 15.0, 18.0, 40);
    add_road(g, 7, 9, 10.0, 14.0,  0);
    add_road(g, 6, 8,  3.0,  7.0,  0);
    add_road(g, 8, 9, 14.0, 28.0,  0);
    add_road(g, 7, 8,  8.0, 12.0,  0);
}

/* ------------------------------------------------------------------------- */
/*  Menu                                                                     */
/* ------------------------------------------------------------------------- */

static void print_menu(void)
{
    printf("\n==================== SmartRoute ====================\n"
           " 1. Show locations and roads\n"
           " 2. Find a route (choose criterion) + explanation\n"
           " 3. Compare all criteria for a trip\n"
           " 4. BFS / DFS traversal from a location\n"
           " 5. Simulate traffic on a road\n"
           " 6. Close / reopen a road\n"
           " 7. Random traffic simulation\n"
           " 8. Reset traffic and closures\n"
           " 9. Add a new location / road\n"
           "10. Set balanced weights (distance/time/cost)\n"
           "11. Export last route to route.json (for a map)\n"
           " 0. Exit\n"
           "====================================================\n");
}

static void pick_trip(const Graph *g, int *src, int *dst)
{
    print_locations(g);
    *src = read_int("Source ID      : ", 0, g->n - 1);
    *dst = read_int("Destination ID : ", 0, g->n - 1);
}

int main(void)
{
    Graph g;
    Weights w = { 1.0 / 3, 1.0 / 3, 1.0 / 3 };
    srand((unsigned)time(NULL));

    graph_init(&g);
    load_sample_city(&g);
    graph_update_norms(&g);

    printf("SmartRoute - Intelligent Multi-Criteria City Route Planner\n"
           "Sample road network with simulated traffic (no live navigation).\n");

    int choice;
    do {
        print_menu();
        choice = read_int("Choose an option: ", 0, 11);

        switch (choice) {
        case 1:
            print_locations(&g);
            print_roads(&g);
            break;

        case 2: {
            int s, d;
            pick_trip(&g, &s, &d);
            printf("\nCriterion: 1) Distance  2) Time  3) Cost  4) Balanced\n");
            Mode m = (Mode)(read_int("Choice: ", 1, 4) - 1);
            Route r = dijkstra_route(&g, s, d, m, &w);
            print_route(&g, &r);
            if (r.found) {
                Route sh = dijkstra_route(&g, s, d, MODE_DISTANCE, &w);
                explain_route(&g, &r, &sh, m, &w);
                g_last_route = r; g_has_last = 1;
            }
            break;
        }

        case 3: {
            int s, d;
            pick_trip(&g, &s, &d);
            compare_routes(&g, s, d, &w);
            break;
        }

        case 4: {
            print_locations(&g);
            int s = read_int("Start ID: ", 0, g.n - 1);
            printf("\n");
            bfs_order(&g, s);
            printf("\n");
            dfs_order(&g, s);
            break;
        }

        case 5: {
            print_locations(&g);
            int a = read_int("Road end A: ", 0, g.n - 1);
            int b = read_int("Road end B: ", 0, g.n - 1);
            if (!find_edge(&g, a, b)) { printf("No direct road between those locations.\n"); break; }
            double f = read_double("Traffic multiplier (1.0 = free flow, up to 5.0): ", 1.0, 5.0);
            set_traffic(&g, a, b, f);
            printf("Traffic on %s <-> %s set to x%.1f. Re-run a route to see the effect.\n",
                   g.nodes[a].name, g.nodes[b].name, f);
            break;
        }

        case 6: {
            print_locations(&g);
            int a = read_int("Road end A: ", 0, g.n - 1);
            int b = read_int("Road end B: ", 0, g.n - 1);
            Edge *e = find_edge(&g, a, b);
            if (!e) { printf("No direct road between those locations.\n"); break; }
            set_closed(&g, a, b, !e->closed);
            printf("Road %s <-> %s is now %s.\n", g.nodes[a].name, g.nodes[b].name,
                   e->closed ? "CLOSED" : "OPEN");
            break;
        }

        case 7:
            random_traffic(&g);
            break;

        case 8:
            reset_conditions(&g);
            printf("All roads reset to free flow and open.\n");
            break;

        case 9: {
            printf("1) Add location   2) Add road : ");
            int t = read_int("", 1, 2);
            if (t == 1) {
                char name[NAME_LEN];
                printf("Name: ");
                read_line(name, sizeof(name));
                if (name[0] == '\0') { printf("Name cannot be empty.\n"); break; }
                double lat = read_double("Latitude : ", -90, 90);
                double lon = read_double("Longitude: ", -180, 180);
                int id = add_node(&g, name, lat, lon);
                if (id < 0) printf("Maximum number of locations reached.\n");
                else printf("Added '%s' with ID %d.\n", name, id);
            } else {
                print_locations(&g);
                int a = read_int("Road end A: ", 0, g.n - 1);
                int b = read_int("Road end B: ", 0, g.n - 1);
                double km   = read_double("Distance (km)      : ", 0.1, 1000);
                double mins = read_double("Base time (min)    : ", 0.1, 1000);
                double toll = read_double("Toll (Rs, 0 = none): ", 0, 10000);
                if (add_road(&g, a, b, km, mins, toll)) printf("Road added.\n");
                else printf("Could not add road (same location or road already exists).\n");
            }
            break;
        }

        case 10: {
            printf("Enter relative importance (0-10) for the balanced criterion.\n");
            double a = read_double("Distance importance: ", 0, 10);
            double b = read_double("Time importance    : ", 0, 10);
            double c = read_double("Cost importance    : ", 0, 10);
            double sum = a + b + c;
            if (sum <= 0) { printf("At least one weight must be above zero.\n"); break; }
            w.wd = a / sum; w.wt = b / sum; w.wc = c / sum;
            printf("Balanced weights -> distance %.0f%%, time %.0f%%, cost %.0f%%\n",
                   w.wd * 100, w.wt * 100, w.wc * 100);
            break;
        }

        case 11:
            if (g_has_last) export_json(&g, &g_last_route, "route.json");
            else printf("No route yet. Use option 2 first.\n");
            break;

        default:
            break;
        }
    } while (choice != 0);

    graph_free(&g);
    printf("Thank you for using SmartRoute.\n");
    return 0;
}