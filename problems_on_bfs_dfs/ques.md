Number of Provinces
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
Number of Islands
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
Flood Fill Algorithm
