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