/**
    @file Graph.h
    @author Kaushik Dey
    @version 1.0 11/1/22 
*/

#include <bits/stdc++.h>

class Vertex{
    public:
        int value;
        bool visited;
        int st; //starting time
        int fin; //finishing time   
        Vertex* parent;
        Vertex(int val, bool vis); //proper initializer
        Vertex();
};

/**
 * @brief Construct a new Vertex:: Vertex object
 * 
 */
Vertex::Vertex(){
}

/**
 * @brief Construct a new Vertex:: Vertex object
 * 
 * @param val is value of the vertex
 * @param vis says if the vertex is visited or not (default is false)
 */
Vertex::Vertex(int val, bool vis=false){
    value = val;
    visited = vis;
}

/**
 * @brief The node class represents a Node in the
 * adjacency list representation of a graph
 * 
 */
class Node{
    public:
        Vertex* vert; //Vertex the node refers to
        Node* next;
        Node();
        Node(Vertex* v);
        void pushBack(Vertex* v);
        bool search(Vertex* v);
};

/*
*Class Methods
*/

/**
 * @brief Construct a new Node:: Node object
 * 
 */
Node::Node(){
    vert = nullptr;
    next = nullptr;
}

/**
 * 
 * @brief Construct a new Node:: Node object
 * 
 * @param v 
 */
Node::Node(Vertex* v){
    vert =  v;
    next = nullptr;
}

/**
 * @brief adds a vertex to the ll
 * 
 * @param v address to the vertex to be pushed
 */
void Node::pushBack(Vertex* v){
    Node* n = new Node(v);
    Node* curr = this;
    while(curr->next){
        if(curr->vert == v) return;
        curr = curr -> next;
    }
    curr -> next = n;
}

/**
 * @brief searches for a vertex in the linked list
 * 
 * @param v 
 * @return true if v found in ll
 * @return false if not
 */
bool Node::search(Vertex* v){
    Node* n = new Node(v);
    Node* curr = this;
    while(curr){
        if(curr->vert == v) return true;
        curr = curr->next;
    }
    return false;
}



/**
 * @brief The graph class
 * 
 */
class Graph{
    private:
        int n; //Number of Vertices
        Node* adjacencyList; //pointer to adjacency list
        bool allowLoops; //to allow i->i edges
        bool undirected; //to allow undirected graphs
        bool weighted; //to allow weighted graphs
        std::map<std::pair<int,int>,int > edges; //map of edge weights
    public:
        Graph();//default constructor
        Graph(int N, bool bool1, bool bool2, bool bool3); //Constructor
        Graph(int N,int** Arr, bool bool1, bool bool2, bool bool3); //Another Constructor
        Graph cliGraph();
        void dfs(int s, std::vector<Node>& dfsList); //BFS traversal given a source Vertex
        void bfs(int s, std::vector<Node>& bfsList);\
        Graph MSTprim(int refNode);
        Graph MSTkruskal(int refNode);
        void cleanUpVisited(); //Cleans up visited bool for every vertex
        void addEdge(int v,int w); //Edge Creating Mechanism
        void addEdge(int v,int w, int e); //Edge Creating Mechanism
        void edgeType(); //Give Status of every edge in DFS
        bool isEdge(int v, int w); //Edge Detecting Mechanism
        bool isVisited(int x); //Check if a node is visited
        void printVertexTimings();//to print start time and end time for each vertex
        std::vector<std::vector<Node>> stronglyConnectedComponents();//Shows Strongly Connected Components
        std::vector<std::vector<Node>> connectedComponents();//Shows Connected Components
        std::vector<std::vector<Node>> topologicalSort(std::vector<std::vector<Node>> components); //does a topological sort on the connected components
        bool topoPref(std::vector<Node> u, std::vector<Node> v);//checks order of connected vertices
        Graph reverseGraph(); //Holy Shit just reverse all edges
};

/**
 * @brief Construct a new Graph:: Graph object
 * Initializes the private members of Graph class
 * @param N Number of Vertices in the graph
 * @param bool1 Allow Loops i->i in a graph?
 * @param bool2 Is the Graph Undirected?
 */
Graph::Graph(int N, bool bool1=true, bool bool2=false, bool bool3=false){
    n = N;
    allowLoops = bool1;
    weighted = bool3;
    undirected = bool2;
    Vertex* vertexArray = new Vertex[n]; //create an array of vertices to be used later
    adjacencyList = new Node[n]; //initialize the adjacencyList to void
    for(int i = 0; i < n; i++){
        vertexArray[i].value = i; //create the vertex
        vertexArray[i].visited = false; //create the vertex
        vertexArray[i].st = 0;
        vertexArray[i].fin = 0;
        adjacencyList[i].vert = vertexArray+i; //assign the head of lists to respective vertices
    }
}

/**
 * @brief Construct a new Graph:: Graph object
 * Initializes the private members of Graph class
 * @param N Number of Vertices in the graph
 * @param Arr Graph Adjacency Matrix
 * @param bool1 Allow Loops i->i in a graph?
 * @param bool2 Is the Graph Undirected?
 */
Graph::Graph(int N, int** Arr, bool bool1=false, bool bool2=false, bool bool3=false){
    n = N;
    allowLoops = bool1;
    undirected = bool2;
    weighted = bool3;
    Vertex* vertexArray = new Vertex[n]; //create an array of vertices to be used later
    adjacencyList = new Node[n]; //initialize the adjacencyList to void
    for(int i = 0; i < n; i++){
        vertexArray[i].value = i; //create the vertex
        vertexArray[i].visited = false; //create the vertex
        vertexArray[i].st = 0;
        vertexArray[i].fin = 0;
        adjacencyList[i].vert = vertexArray+i; //assign the head of lists to respective vertices
    }
    for(int i = 0; i < N; i++){
      for(int j = 0; j < N; j++){
        if(Arr[i][j]==1){
          addEdge(i,j);
        }
      }
    }
}

Graph::Graph(){
    int v1, v2, w;
    std::cout << "Enter the number of vertices: ";
    try{
        std::cin >> n;
        std::cout << "Do you want it to be undirected(0/1): ";
        std::cin >> undirected;
        std::cout << "Do you want it to have loops(0/1): ";
        std::cin >> allowLoops;
        std::cout << "Do you want it to have edge weights(0/1): ";
        std::cin >> weighted;
        std::cout << "You may start entering edges from next line(-1 to exit):" << std::endl;
    }
    catch(...){
        std::cout << "Unspecified Error!!" << std::endl;
    }
    Vertex* vertexArray = new Vertex[n]; //create an array of vertices to be used later
    adjacencyList = new Node[n]; //initialize the adjacencyList to void
    for(int i = 0; i < n; i++){
        vertexArray[i].value = i; //create the vertex
        vertexArray[i].visited = false; //create the vertex
        vertexArray[i].st = 0;
        vertexArray[i].fin = 0;
        adjacencyList[i].vert = vertexArray+i; //assign the head of lists to respective vertices
    }
    while(true){
        try{
            std::cin >> v1;
            if(v1 == -1)break;
            std::cin >> v2;
            if(v1 > n or v2 > n){
                throw 0x001;
            }
            if(weighted){
                std::cin >> w;
                addEdge(v1-1,v2-1,w);
            }
            else{
                addEdge(v1-1,v2-1);
            }
        }
        catch(int err){
            std::cout << "Err No. 0x";
            std::cout << err;
            std::cout << " : ";
            if(err == 0x001){
                std::cout << " Vertex out of bounds" <<std::endl; 
            }
            std::cout << "Try Again" << std::endl;
        }
        catch(...){
            std::cout  << "Err No. " << 0x000 << " : Unidentified Error" << std::endl;
        }
    }
}

/**
 * @brief Add new edges to the graph, v to w
 * The function adds edges by pushing w to a std::vector of v
 * 
 * @param v address to source Node
 * @param w address to sink Node
 */
void Graph::addEdge(int v, int w, int e){
    if(!isEdge(v,w)){
      if(!allowLoops&& v == w) return;
      if(undirected && !isEdge(w,v)){
        adjacencyList[w].pushBack(adjacencyList[v].vert);
        edges.insert( {{w,v},e} );
      };
      adjacencyList[v].pushBack(adjacencyList[w].vert);
      edges.insert( {{v,w},e} );
    }
}

void Graph::addEdge(int v, int w){
    if(!isEdge(v,w)){
      if(!allowLoops&& v == w) return;
      if(undirected && !isEdge(w,v)){
        adjacencyList[w].pushBack(adjacencyList[v].vert);
        edges.insert( {{w,v},1} );
      };
      adjacencyList[v].pushBack(adjacencyList[w].vert);
      edges.insert( {{v,w},1} );
    }
}

/**
 * @brief Depth First Search
 * The function works recursively
 * 
 * @param s the vertex to start from 
 * @param dfsList the list of vertices in search
 */
void Graph::dfs(int s, std::vector<Node> &dfsList){
    static int timer = 1;
    Node* currNode = &adjacencyList[s]; //Defining Root rn
    if(!currNode->vert->visited){ //if this node is not visited
      dfsList.push_back(*currNode);
      currNode->vert->visited = true; //making it visited
    }
    else{ //if visited
      if(dfsList.empty()) return; //terminate
    }
    //std::cout << s << " -> ";
    if(s == 0)currNode->vert->st = timer; //initiation
    Vertex* vp = currNode->vert; //create a parent vertex before moving to next    
    while(currNode->next){ //if node exists
        currNode = currNode->next; //go to next in ll
        if(currNode->vert->visited){ //but if visited
            continue; //check next element
        }
        else{ //if legit
            currNode->vert->parent = vp; //assign parent
            timer++; //increase timer
            currNode->vert->st = timer; //add start timer
            dfs(currNode->vert->value, dfsList); //run dfs recursively
            timer++;//increase timer
        }
    }
    vp->fin = timer; //add finish timer to parent
}

void Graph::bfs(int s,std::vector<Node>& bfsList){
    static std::vector<Node> queue;
    Node* currNode = &adjacencyList[ s ];//define current node as the first element in queue
    if(bfsList.empty())queue.push_back(*currNode); //for first case
    if(queue.empty())return; //terminate when there are no more elements in queue
    currNode->vert->visited = true; //set it to visited
    bfsList.push_back(*currNode); //add to bfsList
    currNode = currNode->next;
    while(currNode){
        if(currNode->vert->visited){
            currNode = currNode->next;
            continue;
        }
        else{ //if not visited
            currNode->vert->visited = true; //set it to visited
            queue.push_back(*currNode);
        }
    }
    if(!queue.empty())queue.erase(queue.begin()); //delete that element
    bfs(queue.front().vert->value, bfsList);
}

Graph Graph::MSTprim(int refNode){
  int nodeweights[n];
  Graph MST(n,false,true,true); //undirected, weighted graph
  for(int i = 0; i < n; i++){
    nodeweights[i] = INT_MAX;
  }
  bool done = false;
  nodeweights[refNode] = 0;
  Node* currNode = &adjacencyList[refNode];
  currNode->vert->visited = true;
  int temp = INT_MAX;
  std::pair<int, int> minNodes;//to hold vertex pair with minimum weight edge
  std::vector<int> MSTstack;
  MSTstack.push_back(refNode);//
  while(MSTstack.size() != n){
    for(auto node : MSTstack){//for every node in the stack
      Node* currNode = &adjacencyList[node];//add currNode to stack
      while(currNode->next){
        currNode = currNode->next;
        if(currNode->vert->visited){
          continue;
        }
        //update the node weights
        nodeweights[currNode->vert->value] = (nodeweights[currNode->vert->value] < nodeweights[node] + edges.find({currNode->vert->value,node})->second) ? nodeweights[currNode->vert->value] : nodeweights[node] + edges.find({currNode->vert->value,node})->second;
        if(temp > nodeweights[currNode->vert->value]){
          temp = nodeweights[currNode->vert->value];
          minNodes = {node,currNode->vert->value}; //reset to minimum node possible
        }
      }
    }
    // std::cout << "Min Nodes: " << minNodes.first << " " << minNodes.second << std::endl;
    MST.addEdge(minNodes.first,minNodes.second,edges.find(minNodes)->second);//add the edge to the MST
    MSTstack.push_back(minNodes.second);
    adjacencyList[minNodes.second].vert->visited = true; //mark it as visited tho
  }
  return MST;
}

/**
 * @brief cleans up visited values of all vertices to false
 * 
 */
void Graph::cleanUpVisited(){
    for(int i = 0; i < n; i++){
        adjacencyList[i].vert->visited = false;
    }
}

/**
 * @brief 
 * 
 */
bool Graph::isVisited(int x){
    return adjacencyList[x].vert->visited;
}

/**
 * @brief It uses the Kosaraju's algorithm to find strongly connected components.
 * @return the vector of vectors representing strongly connected components of a directional graph
 * 
 */
std::vector<std::vector<Node>> Graph::stronglyConnectedComponents(){
    std::vector<Node> dfsList;
    dfs(0,dfsList); //dfs through the graph
    std::vector<std::vector<Node>> connectedList; //create a std::vector of lists
    Graph G = reverseGraph(); //create a fully reversed graph
    int holder; //hold my number
    int num = 1; //sl. no of components
    std::vector<Node> temp;
    for(auto itx : dfsList){
        holder = itx.vert->value; //holder holds the vertex number
        if(G.isVisited(holder)) continue; //check if vertex already visited
        G.dfs(holder, temp);
        connectedList.push_back(
            temp //push back the dfs std::vector
        );
        std::cout << "Component " << num  << ": "; //print
        for( auto i : temp ){
            std::cout << i.vert->value + 1 << " ";
        }
        std::cout << std::endl;
        temp.clear(); //clear up the temporary std::vector
        num++;
    }
    return connectedList;
}

/**
 * @brief prints start time and finish time
 * for each vertex
 * 
 */
void Graph::printVertexTimings(){
    for(int i = 0; i < n; i++){
        std::cout << i << " : (" << adjacencyList[i].vert->st << " , " << adjacencyList[i].vert->fin << " )" << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief This function classifies the edges in the graph
 * according to the dfs tree
 * @return nothing
 */
void Graph::edgeType(){
    Vertex *i_vert, *j_vert; //randomly declared vertices
    for(int i = 0; i < n; i++){ 
        for(int j = 0; j < n; j++){
            if(isEdge(i,j)){ //iterate through i,j
                if(i == j)continue; //no loops around here
                //assign vertices
                i_vert = adjacencyList[i].vert;
                j_vert = adjacencyList[j].vert;
                //std::cout << "\nEdge: " << i << "("<<i_vert->st<<","<<i_vert->fin<<")" << " -> " << j << "("<<j_vert->st<<","<<j_vert->fin<<")" << " : ";
                //Tree Edge and Forward Edge
                if(
                    i_vert->st < j_vert->st &&
                    j_vert->fin < i_vert->fin
                ){
                    if(i_vert == j_vert->parent){
                        std::cout << "Tree Edge";
                    }else{
                        std::cout << "Forward Edge";
                    }
                }
                //Back edge
                if(
                    j_vert->st < i_vert->st &&
                    i_vert->fin < j_vert->fin
                ){
                    std::cout << "Back Edge";
                }
                //Cross Edge
                if(
                    j_vert->fin < i_vert->st
                ){
                    std::cout << "Cross Edge";
                }
            }
        }
    }
}

/**
 * @brief checks if an edge exists between the two edges
 * 
 * @param u first vertex
 * @param v second
 * @return true if u-> v edge exists
 * @return false if it doesnt
 */
bool Graph::isEdge(int u, int v){
    Node n = adjacencyList[u];
    return n.next->search(adjacencyList[v].vert);
}

/**
 * @brief Returns a reversed graph with all directed edges
 * reversed.
 * @return Graph object representing the reversed graph
 */
Graph Graph::reverseGraph(){
    Graph GN(n);
    for(int i = 0; i < n; i++){
        Node* N = (&adjacencyList[i])->next; //the next node
        while(N){ //while next node exists
            GN.addEdge(N->vert->value,i); //add a reverse edge
            N = N -> next;
        }
    }
    return GN; //return the reversed graph
}

/**
 * @brief This takes in all the connected components and sorts them in topological order.
 * Also known as toposort
 * @param components the connected fragments of the graph in vector<Node> format
 * @return std::vector<std::vector<Node>> sorted vector of vector of nodes :)
 */
std::vector<std::vector<Node>> Graph::topologicalSort(std::vector<std::vector<Node>> components){
    Graph G( components.size() );
    int i = 0;
    sort(components.begin(), components.end(), //time for a fancy lambda expression
            [this](const std::vector<Node> lSub, const std::vector<Node> rSub){
                //Should Return true if l < r
                //our motive is to keep higher one before, hence lower
                for(auto itx: lSub){
                    for(auto ity: rSub){
                        if(
                            isEdge( itx.vert->value, ity.vert->value)
                        ){
                            return true; //kill on first edge out
                        }
                        else if(
                            isEdge( ity.vert->value, itx.vert->value)
                        ){
                            return false; //kill on first edge in
                        }
                    }
                }
                return true;
            }
    );
    std::cout << "\n\nTopological Sort: (";
    for(auto i: components){
        std::cout <<"(";
        for(auto j: i){
            std::cout<< j.vert->value+1 <<",";
        }
        std::cout << "),";
    }
    return components;
}


/**
 * @brief 
 *  
 * @return std::vector<std::vector<Node>> 
 */
std::vector<std::vector<Node>> Graph::connectedComponents(){
  std::vector<std::vector<Node>> connectedList;

  if(!undirected){
    std::cout << "ERR: NOT UNDIRECTED GRAPH" << std::endl;
    return connectedList;
  }

  std::vector<Node> dfsList; //temporary list to hold dfs
  for(int i = 0; i < n; i++){
    dfs(i,dfsList);
    if( !dfsList.empty() ){//if the list is not empty
      connectedList.push_back(dfsList);//push back the dfs of node, which will reach all the way since these are double sided
      std::cout << "Community Root : "; //print
      for( auto i : dfsList ){
          std::cout << i.vert->value + 1 << " ";
      }
      std::cout << std::endl;
    }
    dfsList.clear();
  }

  std::cout << "Number of communities: " << connectedList.size() << std::endl;
  return connectedList;
}