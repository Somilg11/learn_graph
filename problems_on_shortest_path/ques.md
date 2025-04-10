## Shortest Path in DAG
```
class Solution {
  private: 
    void topoSort(int node, int vis[], stack<int>& st, vector<pair<int, int>> adj[]){
        vis[node]=1;
        for(auto it: adj[node]){
            int v= it.first;
            if(!vis[v]) topoSort(v, vis, st, adj);
        }
        st.push(node);
    }
  public:
    vector<int> shortestPath(int V, int E, vector<vector<int>>& edges) {
        vector<pair<int,int>> adj[V];
        for(int i=0;i<E;i++){
            int u = edges[i][0];
            int v = edges[i][1];
            int wt = edges[i][2];
            adj[u].push_back({v,wt});
        }
        int vis[V]={0};
        stack<int> st;
        for(int i=0; i<V; i++){
            if(!vis[i]) topoSort(i, vis, st, adj);
        }
        vector<int> dist;
        for(int i=0; i<V; i++) dist.push_back(1e9);
        dist[0]=0;
        while(!st.empty()){
            int node= st.top();
            st.pop();
            
            for(auto it: adj[node]){
                int v= it.first;
                int wt= it.second;
                if(dist[node]+wt<dist[v]) dist[v]=dist[node]+wt;
            }
        }
        for(int i=0; i<V; i++){
            if(dist[i]==1e9) dist[i]=-1;
        }
        return dist;
    }
};
```
## Shortest Path in Undirected
```
class Solution{
public:
    vector<int> shortestPath(vector<vector<int>>& adj, int src) {
        int n = adj.size();
        vector<int>dist(n, INT_MAX);
        queue<int> q;
        dist[src]=0;
        q.push(src);
        while(!q.empty()){
            int node = q.front();
            q.pop();
            for(auto it: adj[node]){
                if(dist[it]==INT_MAX){
                    dist[it] = dist[node]+1;
                    q.push(it);
                }
            }
        }
        for(auto &it: dist){
            if(it == INT_MAX) it = -1;
        }
        return dist;
    }
};
```
## Word Ladder - I
```
#include <bits/stdc++.h>
using namespace std;
class Solution {
public:
    int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
        queue<pair<string, int>> q;
        q.push({beginWord, 1});
        unordered_set<string> st(wordList.begin(), wordList.end());
        st.erase(beginWord);
        while (!q.empty()) {
            string word = q.front().first;
            int steps = q.front().second;
            q.pop();
            if (word == endWord) return steps;
            for (int i = 0; i < word.size(); i++) {
                char orig = word[i];
                for (char ch = 'a'; ch <= 'z'; ch++) {
                    word[i] = ch;
                    if (st.find(word) != st.end()) {
                        st.erase(word);
                        q.push({word, steps + 1});
                    }
                }
                word[i] = orig;
            }
        }
        return 0;
    }
};
```
## Word Ladder - II
```
// first approach
class Solution {
public:
    vector<vector<string>> findLadders(string beginWord, string endWord, vector<string>& wordList) {
        unordered_set<string> st(wordList.begin(), wordList.end());
        queue<vector<string>> q;
        q.push({beginWord});
        vector<string> usedOnLevel;
        usedOnLevel.push_back(beginWord);
        int level = 0;
        vector<vector<string>> ans;
        while(!q.empty()){
            vector<string> vec = q.front();
            q.pop();
            if(vec.size() > level){
                level++;
                for(auto it: usedOnLevel){
                    st.erase(it);
                }
                usedOnLevel.clear();
            }
            string word = vec.back();
            if(word==endWord){
                if(ans.size()==0){
                    ans.push_back(vec);
                }
                else if(ans[0].size() == vec.size()){
                    ans.push_back(vec);
                }
            }
            for(int i=0;i<word.size();i++){
                char og = word[i];
                for(char c = 'a'; c<='z';c++){
                    word[i] = c;
                    if(st.count(word)>0){
                        vec.push_back(word);
                        q.push(vec);
                        usedOnLevel.push_back(word);
                        vec.pop_back();
                    }
                }
                word[i] = og;
            }
        }
        return ans;
    }
};
```
```
// more optimal approach
class Solution {
private:
    void dfs(string word, string beginWord, vector<string>& seq, unordered_map<string, int>& depthMap, vector<vector<string>>& ans) {
        if (word == beginWord) {
            reverse(seq.begin(), seq.end());
            ans.push_back(seq);
            reverse(seq.begin(), seq.end());
            return;
        }
        
        int steps = depthMap[word];
        for (int i = 0; i < word.size(); ++i) {
            char original = word[i];
            for (char ch = 'a'; ch <= 'z'; ++ch) {
                word[i] = ch;
                if (depthMap.count(word) && depthMap[word] + 1 == steps) {
                    seq.push_back(word);
                    dfs(word, beginWord, seq, depthMap, ans);
                    seq.pop_back();
                }
            }
            word[i] = original;
        }
    }
public:
    vector<vector<string>> findLadders(string beginWord, string endWord, vector<string>& wordList) {
        unordered_map<string, int> depthMap;
        vector<vector<string>> ans;
        
        // BFS to find the shortest path
        unordered_set<string> wordSet(wordList.begin(), wordList.end());
        queue<string> q;
        q.push(beginWord);
        depthMap[beginWord] = 1;
        wordSet.erase(beginWord);
        
        while (!q.empty()) {
            string word = q.front();
            q.pop();
            int steps = depthMap[word];
            if (word == endWord) break;
            for (int i = 0; i < word.size(); ++i) {
                char original = word[i];
                for (char ch = 'a'; ch <= 'z'; ++ch) {
                    word[i] = ch;
                    if (wordSet.count(word)) {
                        q.push(word);
                        wordSet.erase(word);
                        depthMap[word] = steps + 1;
                    }
                }
                word[i] = original;
            }
        }
        
        // DFS to find all paths
        if (depthMap.count(endWord)) {
            vector<string> seq = {endWord};
            dfs(endWord, beginWord, seq, depthMap, ans);
        }
        return ans;
    }
};
```
## Dijkstra priority queue
```
class Solution {
  public:
    // Function to find the shortest distance of all the vertices
    // from the source vertex src.
    vector<int> dijkstra(vector<vector<pair<int, int>>> &adj, int src) {
        int V = adj.size();
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        vector<int> dist(V, INT_MAX);
        dist[src]=0;
        pq.push({0,src});
        while(!pq.empty()){
            int dis = pq.top().first;
            int node = pq.top().second;
            pq.pop();
            
            for(auto it: adj[node]){
                int v = it.first;
                int w = it.second;
                if(dis+w < dist[v]){
                    dist[v] = dis+w;
                    pq.push({dis+w,v});
                }
            }
        }
        return dist;
    }
};
```
```
// using set
vector<int> dijkstra(vector<vector<pair<int, int>>> &adj, int src) {
        // Code here
        int V = adj.size();
        set<pair<int, int>> st;
        vector<int> dist(V,INT_MAX);
        st.insert({0,src});
        dist[src]=0;
        while(!st.empty()){
            auto it = *(st.begin()); 
            int node = it.second; 
            int dis = it.first; 
            st.erase(it);
            
            for(auto it: adj[node]){
                int adjNode = it.first; 
                int edgW = it.second;
                if(dis + edgW < dist[adjNode]) {
                    if(dist[adjNode] != 1e9) st.erase({dist[adjNode], adjNode});
                    dist[adjNode] = dis + edgW; 
                    st.insert({dist[adjNode], adjNode}); 
                 }
            }
        }
        return dist;
    }
```
## Shortest path in Binary Matrix
```
class Solution {
public:
    int shortestPathBinaryMatrix(vector<vector<int>>& grid) {
        int n=grid.size();
        int m = grid[0].size();
        if(grid[0][0]!=0 || grid[n-1][m-1]!=0){
            return -1;
        }
         if(grid[0][0]==0 && n==1 && m==1){
            return 1;
        }
        vector<vector<int>> dist(n,vector<int>(m,1e7));
        dist[0][0]=0;
        queue<pair<int,pair<int,int>>> q;
        q.push({1,{0,0}});
        vector<int> drow = {0,1,-1,0,1,-1,1,-1};
        vector<int> dcol = {1,0,0,-1,1,1,-1,-1};
        while(!q.empty()){
            auto it = q.front();
            q.pop();
            int dis = it.first;
            int row = it.second.first;
            int col = it.second.second;
            for(int i=0; i<8; i++){
                int nrow = row+drow[i];
                int ncol = col+dcol[i];
                if(nrow>=0 && nrow<n && ncol>=0 && ncol<m && dis+1<dist[nrow][ncol] && grid[nrow][ncol]==0){
                    dist[nrow][ncol] = dis+1;
                    if(nrow == n-1 && ncol == m-1){
                        return dis+1;
                    }
                    q.push({dis+1,{nrow,ncol}});
                }
            }
        }
        return -1;
    }
};
```
## Path with minimum efforts
```
class Solution {
public:
    int minimumEffortPath(vector<vector<int>>& heights) {
        priority_queue<pair<int, pair<int,int>>, vector<pair<int,pair<int,int>> >,greater<pair<int,pair<int,int>> >> pq;
        int n = heights.size();
        int m = heights[0].size();
        vector<vector<int>> dist(n, vector<int>(m, 1e9));
        dist[0][0]=0;
        pq.push({0, {0,0}});
        int delrow[] = {-1, 0, 1, 0};
        int delcol[] = {0, 1, 0, -1};
        while(!pq.empty()){
            auto it = pq.top();
            pq.pop();
            int diff = it.first;
            int row = it.second.first;
            int col = it.second.second;
            if(row == n-1 && col == m-1) return diff;
            for(int i=0;i<4;i++){
                int newr = row + delrow[i];
                int newc = col + delcol[i];
                if(newr>=0 && newc>=0 && newr<n && newc<m){
                    int newEff = max(abs(heights[row][col] - heights[newr][newc]), diff);
                    if(newEff < dist[newr][newc]){
                        dist[newr][newc] = newEff;
                        pq.push({newEff, {newr, newc}});
                    }
                }
            }
        }
        return 0;
    }
};
```
## Cheapest Flight with K stops
```
class Solution {
public:
    int findCheapestPrice(int n, vector<vector<int>>& flights, int src, int dst, int k) {
        vector<vector<pair<int,int>>>adj(n);
        for(auto it: flights){
            adj[it[0]].push_back({it[1], it[2]});
        }
        queue<pair<int, pair<int,int>> >q;
        q.push({0, {src,0}});
        vector<int> dist(n, 1e9);
        dist[src]=0;
        while(!q.empty()){
            auto it = q.front();
            q.pop();
            int stop = it.first;
            int node = it.second.first;
            int cost = it.second.second;
            if(stop > k) continue;
            for(auto it: adj[node]){
                int adjnode = it.first;
                int edw = it.second;
                if(cost+edw < dist[adjnode] && stop<=k){
                    dist[adjnode] = cost+edw;
                    q.push({stop+1 , {adjnode, cost+edw}});
                }
            }
        }
        if(dist[dst] == 1e9) return -1;
        return dist[dst];
    }
};
```
## Minimum Multiplication to react end
```
class Solution {
  public:
    int minimumMultiplications(vector<int>& arr, int start, int end) {
        // code here
        queue<pair<int,int>> q;
        q.push({start, 0});
        vector<int> dist(100000, 1e9);
        dist[start]=0;
        int mod = 100000;
        if(start == end) return 0;
        while(!q.empty()){
            int node = q.front().first;
            int steps = q.front().second;
            q.pop();
            for(auto it: arr){
                int num = (it*node)%mod;
                if(steps+1<dist[num]){
                    dist[num]=steps+1;
                    if(num == end) return steps+1;
                    q.push({num, steps+1});
                }
            }
        }
        return -1;
    }
};
```
## Number of ways to arrive to destination
```
class Solution {
public:
    int countPaths(int n, vector<vector<int>>& roads) {
        vector<vector<pair<int, int>>> graph(n);
        for (const auto& road : roads) {
            int u = road[0], v = road[1], time = road[2];
            graph[u].emplace_back(v, time);
            graph[v].emplace_back(u, time);
        }
        vector<long long> dist(n, LLONG_MAX);
        vector<int> ways(n, 0);

        dist[0] = 0;
        ways[0] = 1;

        priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> pq;
        pq.emplace(0, 0);
        const int MOD = 1e9 + 7;
        while (!pq.empty()) {
            auto [d, node] = pq.top();
            pq.pop();
            if (d > dist[node]) continue;
            for (const auto& [neighbor, time] : graph[node]) {
                if (dist[node] + time < dist[neighbor]) {
                    dist[neighbor] = dist[node] + time;
                    ways[neighbor] = ways[node];
                    pq.emplace(dist[neighbor], neighbor);
                } else if (dist[node] + time == dist[neighbor]) {
                    ways[neighbor] = (ways[neighbor] + ways[node]) % MOD;
                }
            }
        }
        return ways[n - 1];
    }
};
```
## Bellman-Ford
```
vector<int> bellmanFord(int V, vector<vector<int>>& edges, int src) {
        // Code here
        vector<int> dist(V,1e8);
        dist[src]=0;
        for(int i=0;i<V-1;i++){
            for(auto it: edges){
                int u = it[0];
                int v = it[1];
                int wt = it[2];
                if(dist[u] != 1e8 && dist[u]+wt < dist[v]){
                    dist[v] = dist[u]+wt;
                }
            }
        }
        for(auto it: edges){
                int u = it[0];
                int v = it[1];
                int wt = it[2];
                if(dist[u] != 1e8 && dist[u]+wt < dist[v]) return {-1};
            }
        return dist;
    }
```
## Floyd Warshall
```
class Solution {
  public:
    void shortestDistance(vector<vector<int>>& mat) {
        // Code here
        int n =mat.size();
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                if(mat[i][j]==-1)mat[i][j]=1e9;
                if(i==j)mat[i][j]=0;
            }
        }
        
        for(int k=0;k<n;k++){
            for(int i=0;i<n;i++){
                for(int j=0;j<n;j++){
                    mat[i][j] = min(mat[i][j], mat[i][k] + mat[k][j]);
                }
            }
        }
        
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                if(mat[i][j]==1e9)mat[i][j]=-1;
            }
        }
    }
};
```
## Find the City With the Smallest Number of Neighbors at a Threshold Distance
```
class Solution {
public:
    int findTheCity(int n, vector<vector<int>>& edges, int distanceThreshold) {
        vector<vector<int>>dist(n, vector<int> (n,INT_MAX));
        for(auto it:edges){
            dist[it[0]][it[1]] = it[2];
            dist[it[1]][it[0]] = it[2];
        }
        for(int i=0;i<n;i++) dist[i][i]=0;
        for(int k=0;k<n;k++){
            for(int i=0;i<n;i++){
                for(int j=0;j<n;j++){
                    if(dist[i][k] == INT_MAX || dist[k][j]==INT_MAX) continue;
                    dist[i][j] = min(dist[i][j], dist[i][k]+dist[k][j]);
                }
            }
        }
        int cnt = n;
        int cityno = -1;
        for(int i=0;i<n;i++){
            int count = 0;
            for(int j=0;j<n;j++){
                if(dist[i][j] <= distanceThreshold) count++;
            }
            if(count <= cnt){
                cnt = count;
                cityno = i;
            }
        }
        return cityno;
    }
};
```
## Prim's algo
```
class Solution {
  public:
    // Function to find sum of weights of edges of the Minimum Spanning Tree.
    int spanningTree(int V, vector<vector<int>> adj[]) {
        // code here
        priority_queue<pair<int,int>,
        vector<pair<int,int> >, greater<pair<int,int>>>pq;
        vector<int> vis(V,0);
        pq.push({0,0});
        int sum = 0;
        while(!pq.empty()){
            auto it = pq.top();
            pq.pop();
            int node = it.second;
            int wt = it.first;
            if(vis[node] == 1)continue;
            vis[node]=1;
            sum+=wt;
            for(auto it: adj[node]){
                int adjNode = it[0];
                int edw = it[1];
                if(!vis[adjNode]){
                    pq.push({edw, adjNode});
                }
            }
        }
        return sum;
    }
};
```
## Disjoint set union
```
class DisjointSet{
    vector<int> rank, parent;
public:
    DisjointSet(int n){
        rank.resize(n+1,0);
        parent.resize(n+1);
        for(int i=0;i<=n;i++){
            parent[i]=i;
        }
    }
    int findUPar(int node){
        if(node == parent[node]) return node;
        return parent[node]=findUPar(parent[node]);
    }
    void unionByRank(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
        } else if(rank[ulp_v] < rank[ulp_u]){
            parent[ulp_v] = ulp_u;
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u]++;
        }
    }
    void unionBySize(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
            rank[ulp_v] += rank[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u] += rank[ulp_v];
        }
    }
}
```
## Kruskal Algo
```
class DisjointSet{
    vector<int> rank, parent;
public:
    DisjointSet(int n){
        rank.resize(n+1,0);
        parent.resize(n+1);
        for(int i=0;i<=n;i++){
            parent[i]=i;
        }
    }
    int findUPar(int node){
        if(node == parent[node]) return node;
        return parent[node]=findUPar(parent[node]);
    }
    void unionByRank(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
        } else if(rank[ulp_v] < rank[ulp_u]){
            parent[ulp_v] = ulp_u;
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u]++;
        }
    }

    void unionBySize(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(size[ulp_u] < size[ulp_v]){
            parent[ulp_u] = ulp_v;
            size[ulp_v] += size[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            size[ulp_u] += size[ulp_v];
        }
    }
}

int spanningTree(int V, vector<vector<int>> adj[]){
    vector<pair<int,int>> edges;
    for(int i=0;i<V;i++){
        for(auto it: adj[i]){
            int adjNode = it[0];
            int wt = it[1];
            int node = i;

            edges.push_back({wt, {node, adjNode}});
        }
    }
    DisjointSet ds(V);
    sort(edges.begin(), edges.end());
    int mstWt = 0;
    for(auto it: edges){
        int wt = it.first;
        int u = it.second.first;
        int v = it.second.second;

        if(ds.findUPar(u) != ds.findUPar(v)){
            mstWt+=wt;
            ds.unionBySize(u, v);
        }
    }
    return mstWt;
}
```
## Number of operations to make network connected
```
class DisjointSet{
public:
    vector<int> rank, parent;
    DisjointSet(int n){
        rank.resize(n+1,0);
        parent.resize(n+1);
        for(int i=0;i<=n;i++){
            parent[i]=i;
        }
    }
    int findUPar(int node){
        if(node == parent[node]) return node;
        return parent[node]=findUPar(parent[node]);
    }
    void unionBySize(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
            rank[ulp_v] += rank[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u] += rank[ulp_v];
        }
    }
};
class Solution {
public:
    int makeConnected(int n, vector<vector<int>>& edges) {
        DisjointSet ds(n);
        int countExt = 0;
        for(auto it: edges){
            int u = it[0];
            int v = it[1];
            if(ds.findUPar(u) == ds.findUPar(v)){
                countExt++;
            } else ds.unionBySize(u, v);
        }
        int cntC = 0;
        for(int i=0;i<n;i++){
            if(ds.parent[i] == i) cntC++;
        }
        int ans = cntC - 1;
        if(countExt >= ans)return ans;
        return -1;
    }
};
```
## Accounts Merged
```
class DisjointSet{
public:
    vector<int> rank, parent;
    DisjointSet(int n){
        rank.resize(n+1,0);
        parent.resize(n+1);
        for(int i=0;i<=n;i++){
            parent[i]=i;
        }
    }
    int findUPar(int node){
        if(node == parent[node]) return node;
        return parent[node]=findUPar(parent[node]);
    }
    void unionBySize(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
            rank[ulp_v] += rank[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u] += rank[ulp_v];
        }
    }
};
class Solution {
public:
    vector<vector<string>> accountsMerge(vector<vector<string>>& accounts) {
        int n = accounts.size();
        DisjointSet ds(n);
        unordered_map<string,int> mpp;
        for(int i=0;i<n;i++){
            for(int j=1;j<accounts[i].size();j++){
                string mail = accounts[i][j];
                if(mpp.find(mail) == mpp.end()){
                    mpp[mail]=i;
                } else {
                    ds.unionBySize(i, mpp[mail]);
                }
            }
        }
        vector<string> ls[n];
        for(auto it: mpp){
            string mail = it.first;
            int node = ds.findUPar(it.second);
            ls[node].push_back(mail);
        }

        vector<vector<string>> ans;
        for(int i=0;i<n;i++){
            if(ls[i].size() == 0) continue;
            sort(ls[i].begin(), ls[i].end());
            vector<string> temp;
            temp.push_back(accounts[i][0]);
            for(auto it: ls[i]){
                temp.push_back(it);
            }
            ans.push_back(temp);
        }
        return ans;
    }
};
```
## Maximum connected groups
```
class DisjointSet {
public:
    vector<int> parent, size;

    DisjointSet(int n) {
        parent.resize(n);
        size.resize(n, 1);
        for(int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    int findUPar(int node) {
        if(node == parent[node]) return node;
        return parent[node] = findUPar(parent[node]);
    }

    void unionBySize(int u, int v) {
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;

        if(size[ulp_u] < size[ulp_v]) {
            parent[ulp_u] = ulp_v;
            size[ulp_v] += size[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            size[ulp_u] += size[ulp_v];
        }
    }
};

class Solution {
private:
    bool isValid(int r, int c, int n) {
        return r >= 0 && r < n && c >= 0 && c < n;
    }

public:
    int largestIsland(vector<vector<int>>& grid) {
        int n = grid.size();
        DisjointSet ds(n * n);

        // Step 1: Union all connected 1s
        for(int row = 0; row < n; ++row) {
            for(int col = 0; col < n; ++col) {
                if(grid[row][col] == 0) continue;
                int node = row * n + col;
                int dr[] = {-1, 0, 1, 0};
                int dc[] = {0, -1, 0, 1};

                for(int i = 0; i < 4; ++i) {
                    int newr = row + dr[i];
                    int newc = col + dc[i];
                    if(isValid(newr, newc, n) && grid[newr][newc] == 1) {
                        int adjNode = newr * n + newc;
                        ds.unionBySize(node, adjNode);
                    }
                }
            }
        }

        // Step 2: Try converting each 0 to 1 and compute connected component size
        int maxIsland = 0;
        for(int row = 0; row < n; ++row) {
            for(int col = 0; col < n; ++col) {
                if(grid[row][col] == 1) continue;

                int dr[] = {-1, 0, 1, 0};
                int dc[] = {0, -1, 0, 1};
                set<int> uniqueParents;
                for(int i = 0; i < 4; ++i) {
                    int newr = row + dr[i];
                    int newc = col + dc[i];
                    if(isValid(newr, newc, n) && grid[newr][newc] == 1) {
                        int parent = ds.findUPar(newr * n + newc);
                        uniqueParents.insert(parent);
                    }
                }

                int newSize = 1; // For the converted 0
                for(int parent : uniqueParents) {
                    newSize += ds.size[parent];
                }
                maxIsland = max(maxIsland, newSize);
            }
        }

        // Step 3: If the grid was all 1s already
        for(int i = 0; i < n * n; ++i) {
            maxIsland = max(maxIsland, ds.size[ds.findUPar(i)]);
        }

        return maxIsland;
    }
};
```
## Most stones removed with the same row-col
```
class DisjointSet{
public:
    vector<int> rank, parent;
    DisjointSet(int n){
        rank.resize(n+1,0);
        parent.resize(n+1);
        for(int i=0;i<=n;i++){
            parent[i]=i;
        }
    }
    int findUPar(int node){
        if(node == parent[node]) return node;
        return parent[node]=findUPar(parent[node]);
    }
    void unionBySize(int u, int v){
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if(ulp_u == ulp_v) return;
        if(rank[ulp_u] < rank[ulp_v]){
            parent[ulp_u] = ulp_v;
            rank[ulp_v] += rank[ulp_u];
        } else {
            parent[ulp_v] = ulp_u;
            rank[ulp_u] += rank[ulp_v];
        }
    }
};
class Solution {
public:
    int removeStones(vector<vector<int>>& stones) {
        int n =stones.size();
        int maxRow = 0;
        int maxCol = 0;
        for(auto it: stones){
            maxRow = max(maxRow, it[0]);
            maxCol = max(maxCol, it[1]);
        }
        DisjointSet ds(maxRow + maxCol + 2);
        unordered_map<int,int> stoneNode;
        for(auto it: stones){
            int nodeRow = it[0];
            int nodeCol = it[1] + maxRow + 1;
            ds.unionBySize(nodeRow, nodeCol);
            stoneNode[nodeRow] = 1;
            stoneNode[nodeCol] = 1;
        }
        int cnt = 0;
        for(auto it: stoneNode){
            if(ds.findUPar(it.first) == it.first) cnt++;
        }
        return n-cnt;
    }
};
```
## Strongly connected components - Kosaraju's algo
```
class Solution {
  private:
  void dfs(int node, vector<int> &vis, vector<vector<int>> &adj, stack<int> &st){
      vis[node] = 1;
      for(auto it: adj[node]){
          if(!vis[it]) dfs(it, vis, adj, st);
      }
      st.push(node);
  }
  private:
  void dfs3(int node, vector<int> &vis, vector<vector<int>> &adjT){
      vis[node] = 1;
      for(auto it: adjT[node]){
          if(!vis[it]) dfs3(it, vis, adjT);
      }   
  }
  public:
    int kosaraju(vector<vector<int>> &adj) {
        // code here
        int n = adj.size();
        vector<int> vis(n, 0);
        stack<int> st;
        for(int i=0;i<n;i++){
            if(!vis[i]){
                dfs(i, vis, adj, st);
            }
        }
        vector<vector<int>> adjT(n);
        for(int i=0;i<n;i++){
            vis[i]=0;
            for(auto it: adj[i]){
                adjT[it].push_back(i);
            }
        }
        int scc = 0;
        while(!st.empty()){
            int node = st.top();
            st.pop();
            if(!vis[node]){
                scc++;
                dfs3(node, vis, adjT);
            }
        }
        return scc;
    }
};
```