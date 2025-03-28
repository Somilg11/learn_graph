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
## 0/1 Matrix
```
class Solution {
public:
    vector<vector<int>> updateMatrix(vector<vector<int>>& grid) {
        int n = grid.size(); 
	    int m = grid[0].size();
	    vector<vector<int>> vis(n, vector<int>(m, 0)); 
	    vector<vector<int>> dist(n, vector<int>(m, 0));
	    queue<pair<pair<int,int>, int>> q;
	    for(int i = 0;i<n;i++) {
	        for(int j = 0;j<m;j++) {
	            if(grid[i][j] == 0) {
	                q.push({{i,j}, 0}); 
	                vis[i][j] = 1; 
	            }
	            else {
	                vis[i][j] = 0; 
	            }
	        }
	    }
	    
	    int delrow[] = {-1, 0, +1, 0}; 
	    int delcol[] = {0, +1, 0, -1}; 
	    while(!q.empty()) {
	        int row = q.front().first.first; 
	        int col = q.front().first.second; 
	        int steps = q.front().second; 
	        q.pop(); 
	        dist[row][col] = steps; 
	        for(int i = 0;i<4;i++) {
	            int nrow = row + delrow[i]; 
	            int ncol = col + delcol[i]; 
	            if(nrow >= 0 && nrow < n && ncol >= 0 && ncol < m 
	            && vis[nrow][ncol] == 0) {
	                vis[nrow][ncol] = 1; 
	                q.push({{nrow, ncol}, steps+1});  
	            }
	        }
	    }
	    return dist; 
    }
};
```
## Surrounded Regions
```
class Solution {
public:
    void dfs(vector<vector<char>>& board, int i, int j, int m, int n){
        if(i<0 or j<0 or i>=m or j>=n or board[i][j] != 'O') return;
        board[i][j] = '#';
        dfs(board, i-1, j, m, n);
        dfs(board, i+1, j, m, n);
        dfs(board, i, j-1, m, n);
        dfs(board, i, j+1, m, n);
    }
    void solve(vector<vector<char>>& board) {
        int m = board.size();
        int n = board[0].size();
        if(m==0)return;
        for(int i=0;i<m;i++){
            if(board[i][0] == 'O') dfs(board, i, 0, m, n);
            if(board[i][n-1] == 'O') dfs(board, i, n-1, m, n);
        }
        for(int j=0;j<n;j++){
            if(board[0][j] == 'O') dfs(board, 0, j, m, n);
            if(board[m-1][j] == '0') dfs(board, m-1, j, m, n);
        }
        for(int i=0;i<m;i++){
            for(int j=0;j<n;j++){
                if(board[i][j] == 'O')  board[i][j]='X';
                if(board[i][j] == '#')  board[i][j]='O';
            }
        }
    }
};
```
## Number of Enclaves
```
class Solution {
public:
    int numEnclaves(vector<vector<int>>& grid) {
        queue<pair<int,int>> q;
        int n = grid.size();
        int m = grid[0].size();
        vector<vector<int>> vis(n, vector<int>(m, 0));
        for(int i=0;i<n;i++){
            for(int j=0;j<m;j++){
                if(i==0 || j==0 || i==n-1 || j==m-1){
                    if(grid[i][j]==1){
                        q.push({i,j});
                        vis[i][j]=1;
                    }
                }
            }
        }
        int delrow[] = {-1, 0, 1, 0};
        int delcol[] = {0, 1, 0, -1};
        
        while(!q.empty()){
            int row = q.front().first;
            int col = q.front().second;
            q.pop();
            for(int i=0;i<4;i++){
                int nrow = row+delrow[i];
                int ncol = col+delcol[i];
                if(nrow>=0 && nrow<n && ncol>=0 && ncol<m && vis[nrow][ncol] == 0 && grid[nrow][ncol]==1){
                    q.push({nrow, ncol});
                    vis[nrow][ncol]=1;
                }
            }
        }
        int cnt = 0;
        for(int i=0;i<n;i++){
            for(int j=0;j<m;j++){
                if(grid[i][j]==1 && vis[i][j]==0) cnt++;
            }
        }
        return cnt;
    }
};
```
## Bipartite graph
```
//BFS
class Solution {
private:
    bool check(int start, int n, vector<vector<int>>& graph, vector<int> &color){
        queue<int> q;
        q.push(start);
        color[start]=0;
        while(!q.empty()){
            int node = q.front();
            q.pop();
            for(auto it: graph[node]){
                if(color[it]==-1){
                    color[it] = !color[node];
                    q.push(it);
                } else if(color[it]==color[node]){
                    return false;
                }
            }
        }
        return true;
    }
public:
    bool isBipartite(vector<vector<int>>& graph) {
        int n = graph.size();
        vector<int> color(n, -1);
        for(int i=0;i<n;i++){
            if(color[i]==-1){
                if(check(i,n,graph,color) == false) return false;
            }
        }
        return true;
    }
};
```
```
//DFS
class Solution {
private:
    
    bool dfs(int node, int col, vector<vector<int>>& graph, vector<int> &color){
        color[node] = col;
        for(auto it: graph[node]){
            if(color[it]==-1){
                if(dfs(it, !col, graph, color) == false) return false;
            } else if(color[it]==col) return false;
        }
        return true;    
    }
public:
    bool isBipartite(vector<vector<int>>& graph) {
        int n = graph.size();
        vector<int> color(n, -1);
        for(int i=0;i<n;i++){
            if(color[i]==-1){
                if(dfs(i,0,graph,color) == false) return false;
            }
        }
        return true;
    }
};
```
## Detect cycle in directed graph
```
class Solution {
public:
    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
        vector<int> in(numCourses, 0);
        vector<int> topo;
        int n = 0;
        vector<vector<int>> adj(numCourses);
        for(auto it : prerequisites) {
            in[it[0]]++;
            adj[it[1]].push_back(it[0]);
        }
        queue<int> q;
        for(int i = 0; i < numCourses; i++) {
            if(in[i] == 0) q.push(i);
        }
        while(!q.empty()) {
            int node = q.front();
            q.pop();
            topo.push_back(node);
            n++;
            for(auto it : adj[node]) {
                in[it]--;
                if(in[it] == 0) {
                    q.push(it);
                }
            }
        }
        if(n != numCourses) return {};
        return topo;
    }
};
```
## Find Eventual Safe State
```
class Solution {
public:
    bool dfs(int node, vector<vector<int>>& adj, vector<bool>& visit, vector<bool>& inStack) {
        if (inStack[node]) return true;
        if (visit[node]) return false;
        visit[node] = true;
        inStack[node] = true;
        for (auto neighbor : adj[node]) {
            if (dfs(neighbor, adj, visit, inStack)) {
                return true;
            }
        }
        inStack[node] = false;
        return false;
    }
    vector<int> eventualSafeNodes(vector<vector<int>>& graph) {
        int n = graph.size();
        vector<bool> visit(n), inStack(n);
        for (int i = 0; i < n; i++) {
            dfs(i, graph, visit, inStack);
        }
        vector<int> safeNodes;
        for (int i = 0; i < n; i++) {
            if (!inStack[i]) {
                safeNodes.push_back(i);
            }
        }
        return safeNodes;
    }
};
```
## Topological Sort
```
class Solution {
  private:
    void dfs(int start, vector<int> &vis, stack<int> &st, vector<vector<int>> &adj){
        vis[start]=1;
        for(auto it: adj[start]){
            if(!vis[it]){
                dfs(it, vis, st, adj);
            }
        }
        st.push(start);
    }
  public:
    // Function to return list containing vertices in Topological order.
    vector<int> topologicalSort(vector<vector<int>>& adj) {
        // Your code here
        int n = adj.size();
        vector<int> vis(n,0);
        stack<int> st;
        for(int i=0;i<n;i++){
            if(!vis[i]){
                dfs(i, vis, st, adj);
            }
        }
        vector<int> ans;
        while(!st.empty()){
            ans.push_back(st.top());
            st.pop();
        }
        return ans;
    }
};
```
## Kahn's Algorithm
```
vector<int> indegree(n,0);
for(int i=0;i<n;i++){
    for(auto it: adj[i]){
        indegree[it]++;
    }
}
queue<int> q;
for(int i=0;i<n;i++){
    if(indegree[i]==0){
        q.push(i);
    }
}
vector<int> topo;
    while(!q.empty()){
        int node = q.front();
        q.pop();
        topo.push_back(node);
        for(auto it: adj[node]){
            indegree[it]--;
            if(indegree[it]==0) q.push(it);
        }
    }
return topo;
```
## Cycle Detection in Directed Graph BFS
```
class Solution {
  public:
    // Function to detect cycle in a directed graph.
    bool isCyclic(vector<vector<int>> &adj) {
        // code here
        int n = adj.size();
        vector<int> indegree(n,0);
        for(int i=0;i<n;i++){
            for(auto it: adj[i]){
                indegree[it]++;
            }
        }
        queue<int>q;
        for(int i=0;i<n;i++){
            if(indegree[i]==0){
                q.push(i);
            }
        }
        // vector<int> topo;
        int cnt = 0;
        while(!q.empty()){
            int node = q.front();
            q.pop();
            cnt++;
            // topo.push_back(node);
            for(auto it: adj[node]){
                indegree[it]--;
                if(indegree[it]==0) q.push(it);
            }
        }
        if(cnt == n) return false;
        return true;
    }
};
```
## Course Schedule I
```
class Solution {
private:
    bool dfs(int start, vector<int>&vis, vector<int> &st, vector<vector<int>>&adj){
        vis[start] = 1;
        st[start]=1;
        for(auto it: adj[start]){
            if(vis[it]==0){
                if(dfs(it, vis, st, adj)) return true;
            } else if(st[it]) return true;
        }
        st[start]=0;
        return false;
    }
public:
    bool canFinish(int n, vector<vector<int>>& prerequisites) {
        vector<vector<int>> adj(n);
        for (auto& pre : prerequisites) {
            adj[pre[1]].push_back(pre[0]);
        }
        vector<int> vis(n,0);
        vector<int> restack(n,0);
        for(int i=0;i<n;i++){
            if(vis[i]==0){
                if(dfs(i, vis, restack, adj)) return false;
            }
        }
        return true;
    }
};
```
## Course Schedule II
```
class Solution {
public:
    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
        vector<int> in(numCourses, 0);
        vector<int> topo;
        int n = 0;
        vector<vector<int>> adj(numCourses);
        for(auto it : prerequisites) {
            in[it[0]]++;a
            adj[it[1]].push_back(it[0]);
        }
        queue<int> q;
        for(int i = 0; i < numCourses; i++) {
            if(in[i] == 0) q.push(i);
        }
        while(!q.empty()) {
            int node = q.front();
            q.pop();
            topo.push_back(node);
            n++;
            for(auto it : adj[node]) {
                in[it]--;
                if(in[it] == 0) {
                    q.push(it);
                }
            }
        }
        if(n != numCourses) return {};
        return topo;
    }
};
```
## Alien Dictionary - Topo Sort
```
class Solution {
private:
	vector<int> topoSort(int V, vector<int> adj[]){
		int indegree[V] = {0};
		for (int i = 0; i < V; i++) {
			for (auto it : adj[i]) {
				indegree[it]++;
			}
		}
		queue<int> q;
		for (int i = 0; i < V; i++) {
			if (indegree[i] == 0) {
				q.push(i);
			}
		}
		vector<int> topo;
		while (!q.empty()) {
			int node = q.front();
			q.pop();
			topo.push_back(node);
			for (auto it : adj[node]) {
				indegree[it]--;
				if (indegree[it] == 0) q.push(it);
			}
		}
		return topo;
	}
public:
	string findOrder(string dict[], int N, int K) {
		vector<int>adj[K];
		for (int i = 0; i < N - 1; i++) {
			string s1 = dict[i];
			string s2 = dict[i + 1];
			int len = min(s1.size(), s2.size());
			for (int ptr = 0; ptr < len; ptr++) {
				if (s1[ptr] != s2[ptr]) {
					adj[s1[ptr] - 'a'].push_back(s2[ptr] - 'a');
					break;
				}
			}
		}
		vector<int> topo = topoSort(K, adj);
		string ans = "";
		for (auto it : topo) {
			ans = ans + char(it + 'a');
		}
		return ans;
	}
};
```