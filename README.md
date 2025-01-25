# Graph learning

GraphX - Black Box - updated (more refined)

```

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <cmath>

using namespace std;

typedef long long ll;

<!-- ----BLACKBOX STARTS  -->

class Hash {
  private:
    map<vector<ll>, ll> hash_table;
  public:
    Hash() {}
    ll hash(ll x) {
        return hash({x, 0, 0});
    }
    ll hash(vector<ll> x) {
        if (hash_table.find(x) != hash_table.end())
            return hash_table[x];
        ll new_hash = hash_table.size();
        hash_table[x] = new_hash;
        return new_hash;
    }
};

class Graph {
    bool is_directed;

  public:
    vector<vector<pair<ll, ll>>> adj;
    ll n, N = 5000000;
    Hash h;

    Graph(ll n_, bool is_directed_ = true) {
        n = n_;
        is_directed = is_directed_;
        adj.resize(N, vector<pair<ll, ll>>());
    }

    ll hash(ll u, ll v) {
        return h.hash({u, v});
    }
    ll hash(ll u, ll v, ll k) {
        return h.hash({u, v, k});
    }

    void add_edge(ll uR, ll vR, ll c = 0) {
        ll u = h.hash(uR), v = h.hash(vR);
        add_edge_internal(u, v, c);
    }
    void add_edge(vector<ll> uR, vector<ll> vR, ll c = 0) {
        ll u = h.hash(uR), v = h.hash(vR);
        add_edge_internal(u, v, c);
    }

  private:
    void add_edge_internal(ll u, ll v, ll c = 0) {
        add_edge_weighted_undirected(u, v, c);
        if (!is_directed)
            add_edge_weighted_undirected(v, u, c);
    }
    void add_edge_weighted_undirected(ll u, ll v, ll c) {
        pair<ll, ll> p = make_pair(v, c);
        adj[u].push_back(p);
    }
};


class BFS {
    vector<ll> min_dist_from_source;
    vector<bool> visited;
    Graph *g;

  public:
    BFS(Graph *g_) {
        g = g_;
        clear();
    }

    void clear() {
        min_dist_from_source.clear();
        min_dist_from_source.resize(g->N, -1);
        visited.clear();
        visited.resize(g->N, false);
    }

    void run(ll sourceR) {
        ll source = g->h.hash(sourceR);
        run_internal(source);
    }
    void run(vector<ll> sourceR) {
        ll source = g->h.hash(sourceR);
        run_internal(source);
    }

    ll min_dist(ll targetR) {
        ll target = g->h.hash(targetR);
        return min_dist_internal(target);
    }
    ll min_dist(vector<ll> targetR) {
        ll target = g->h.hash(targetR);
        return min_dist_internal(target);
    }

    bool is_visited(ll targetR) {
        ll target = g->h.hash(targetR);
        return is_visited_internal(target);
    }
    bool is_visited(vector<ll> targetR) {
        ll target = g->h.hash(targetR);
        return is_visited_internal(target);
    }

  private:
    void run_internal(ll source) {
        queue<ll> q;
        q.push(source);

        visited[source] = true;
        min_dist_from_source[source] = 0;

        while (!q.empty()) {
            ll cur_node = q.front();
            for (unsigned int i = 0; i < (g->adj[cur_node]).size(); ++i) {
                ll adj_node = (g->adj[cur_node])[i].first;
                if (visited[adj_node] == false) {
                    visited[adj_node] = true;
                    min_dist_from_source[adj_node] = min_dist_from_source[cur_node] + 1;
                    q.push(adj_node);
                }
            }
            q.pop();
        }

        return;
    }

    ll min_dist_internal(ll target) {
        return min_dist_from_source[target];
    }

    bool is_visited_internal(ll target) {
        return visited[target];
    }
};

<!-- ----BLACKBOX ENDS -->

```