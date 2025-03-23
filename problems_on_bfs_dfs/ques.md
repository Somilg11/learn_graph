Number of Provinces
```
class Solution {
private:
    vois bfs(int start, vector<int> &vis, vector<vector<int>> &adj){
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
            }
        }
        return count;
    }
};
```

