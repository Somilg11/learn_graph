```
class Solution {
  private:
    void dfs(vector<vector<int>> &adj, vector<int> &vis, vector<int> &ans, int start){
        vis[start] = 1;
        ans.emplace_back(start);
        for(auto it: adj[start]){
            if(!vis[it]){
                vis[it] = 1;
                dfs(adj, vis, ans, it);
            }
        }
    }
  public:
    // Function to return a list containing the DFS traversal of the graph.
    vector<int> dfsOfGraph(vector<vector<int>>& adj) {
        int start = 0;
        int n = adj.size();
        vector<int> vis(n,0);
        vector<int> ans;
        dfs(adj, vis, ans, start);
        return ans;
    }
};
```