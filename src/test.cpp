#include "Graphs.h"
#include <iostream>

using namespace std;

int main(){
    Graph G1;
    Graph G2 = G1.MSTprim(3);
    for(int i = 0; i < 14; i++){
        for(int j = 0; j < 14; j++){
            cout << i+1 << " to " << j+1 << ": " << G1.isEdge(i,j) << endl;
        }
    }
    G2.cleanUpVisited();
    vector<Node> dfsList;
    G1.dfs(0,dfsList);
    for(auto x:dfsList){
        cout << x.vert->value + 1<< ", ";
    }
}