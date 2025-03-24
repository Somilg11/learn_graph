## Number of Provinces
```
class Solution {
private:
    void dfs(int start, vector<int> &vis, vector<vector<int>> &adj){
        vis[start] = 1;
        for(auto it: adj[start]){
            if(!vis[it]){
                dfs(it, vis, adj);
            }
        }
    }
    void bfs(int start, vector<int> &vis, vector<vector<int>> &adj){
        queue<int> q;
        vis[start] = 1;
        q.push(start);
        while(!q.empty()){
            int node = q.front();
            q.pop();
            for(auto it: adj[node]){
                if(!vis[it]){
                    vis[it] = 1;
                    q.push(it);
                }
            }
        }
    }
public:
    int findCircleNum(vector<vector<int>>& isConnected) {
        int n = isConnected.size();
        vector<vector<int>> adj;
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                adj[i].emplace_back(j);
                adj[j].emplace_back(i);
            }
        }
        vector<int> vis(n,0);
        int count = 0;
        for(int i=0;i<n;i++){
            if(!vis[i]){
                count++;
                bfs(i, vis, adj);
                // dfs(i, vis, adj);
            }
        }
        return count;
    }
};
```
## Number of Islands
```
class Solution{
    private:
        void dfs(int start, vectoR<int> &vis, vector<vector<int>> &adj){
            vsi[start] = 1;
            for(auto it: adj[start]){
                if(!vis[it]) dfs(it, vis, adj);
            }
        }
        void bfs(int start, vector<int> &vis, vector<vector<int>> &adj){
            queue<int>q;
            vis[start]=1;
            q.push(start);
            while(!q.empty()){
                int node = q.front();
                q.pop();
                for(auto it: adj[node]){
                    if(!vis[it]){
                        vis[it] = 1;
                        q.push(it);
                    }
                }
            }
        }
    public:
        int numProvinces(vector<vector<int>> adj, int n) {
            vector<vector<int>> adjL(n);
            for(int i=0;i<n;i++){
                for(int j=0;j<n;j++){
                    if(adj[i][j]==1 && i!=j){
                        adjL[i].emplace_back(j);
                        adjL[j].emplace_back(i);
                    }
                }
            }
            vector<int> vis(n,0);
            int count = 0;
            for(int i=0;i<n;i++){
                if(!vis[i]){
                    count++;
                    bfs(i,vis,adjL);
                    // dfs(i,vis,adjL);
                }
            }
        }
}
```
## Flood Fill Algorithm
```
class Solution {
private:
    void dfs(int row, int col, vector<vector<int>> &ans, vector<vector<int>> &image, int color, int delrow[], int delcol[], int initialcolor){
        int n = image.size();
        int m = image[0].size();
        ans[row][col] = color;
        for(int i=0;i<4;i++){
            int nrow = row + delrow[i];
            int ncol = col + delcol[i];
            if( nrow>=0 && nrow<n && ncol>=0 && ncol<m && image[row][col]==initialcolor && ans[nrow][ncol]!=color){
                dfs(nrow, ncol, ans, image, color, delrow, delcol, initialcolor);
            }
        }
    }
public:
    vector<vector<int>> floodFill(vector<vector<int>>& image, int sr, int sc, int color) {
        int initialcolor = image[sr][sc];
        vector<vector<int>> ans = image;
        int delrow[] = {-1, 0, 1, 0};
        int delcol[] = {0, 1, 0, -1};
        dfs(sr, sc, ans, image, color, delrow, delcol, initialcolor);
        return ans;
    }
};
```
## Rotten Oranges
```
class Solution {
public:
    int orangesRotting(vector<vector<int>>& grid) {
        int n = grid.size();
        int m = grid[0].size();
        
        queue<pair<pair<int, int>, int>> q;
        vector<vector<int>> vis(n, vector<int>(m, 0));

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                if (grid[i][j] == 2) {
                    q.push({{i, j}, 0});
                    vis[i][j] = 1;
                }
            }
        }

        int tm = 0;
        int drow[] = {-1, 0, +1, 0};
        int dcol[] = {0, +1, 0, -1};
        while (!q.empty()) {
            int r = q.front().first.first;
            int c = q.front().first.second;
            int t = q.front().second;
            q.pop();
            tm = max(tm, t);

            for (int i = 0; i < 4; i++) {
                int nrow = r + drow[i];
                int ncol = c + dcol[i];

                if (nrow >= 0 && nrow < n && ncol >= 0 && ncol < m 
                    && grid[nrow][ncol] == 1 && vis[nrow][ncol] == 0) {
                    
                    q.push({{nrow, ncol}, t + 1});
                    grid[nrow][ncol] = 2;
                    vis[nrow][ncol] = 1;
                }
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                if (grid[i][j] == 1) {
                    return -1;
                }
            }
        }
        return tm;
    }
};
```
## Cycle Detection
```
//DFS
class Solution {
  private:
    bool detect(int src,int parent, vector<vector<int>> &adj, vector<int> &vis){
        vis[src] = 1;
        for(auto it: adj[src]){
            if(it==parent) continue;
            if(vis[it]==1) return true;
            else if(detect(it,src,adj,vis)) return true;
        }
        return false;
    }
  public:
    // Function to detect cycle in an undirected graph.
    bool isCycle(vector<vector<int>>& adj) {
        int n = adj.size();
        vector<int> vis(n,0);
        for(int i=0;i<n;i++){
            if(!vis[i] && detect(i,-1, adj, vis)){
                return true;
            }
        }
        return false;
    }
};
```

```
//BFS
class Solution {
  private:
    bool detect(int src, vector<vector<int>> &adj, vector<int> &vis){
        vis[src] = 1;
        queue<pair<int,int>> q; 
        q.push({src, -1});
        while(!q.empty()){
            int node = q.front().first;
            int parent = q.front().second;
            q.pop();
            for(auto it: adj[node]){
                if(!vis[it]){
                    vis[it]=1;
                    q.push({it, node});
                } else if(parent!=it) return true;
            }
        }
        return false;
    }
  public:
    // Function to detect cycle in an undirected graph.
    bool isCycle(vector<vector<int>>& adj) {
        // Code here
        int n = adj.size();
        vector<int> vis(n,0);
        for(int i=0;i<n;i++){
            if(!vis[i] && detect(i, adj, vis)){
                return true;
            }
        }
        return false;
    }
};
```