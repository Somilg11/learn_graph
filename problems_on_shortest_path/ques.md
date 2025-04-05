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