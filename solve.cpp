#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Graph {
  public:
    using Vertex = size_t;
    using VertexSet = std::unordered_set<Vertex>;
    using AdjacencyList = std::unordered_map<Vertex, VertexSet>;

    void AddVertex(Vertex v) {
        adjacency_list_[v];
    }

    void AddEdge(Vertex u, Vertex v) {
        adjacency_list_[u].insert(v);
        adjacency_list_[v].insert(u);
    }

    const VertexSet& AdjacentVertices(Vertex v) const {
        const auto it = adjacency_list_.find(v);
        if (it != adjacency_list_.end()) {
            return it->second;
        } else {
            return empty_set_;
        }
    }

    VertexSet AllVertices() const {
        VertexSet vs;
        vs.reserve(adjacency_list_.size());
        for (const auto& pair : adjacency_list_) {
            const auto& vertex = pair.first;
            vs.insert(vertex);
        }
        return vs;
    }

    const AdjacencyList& AsAdjacencyList() const {
        return adjacency_list_;
    }

    static const VertexSet empty_set_;
  private:
    AdjacencyList adjacency_list_;
};

const Graph::VertexSet Graph::empty_set_;

class VertexCut {
  public:
    explicit VertexCut(const Graph& graph)
        : graph_(graph), set_A_(graph.AllVertices()), set_B_(Graph::empty_set_) {}

    const Graph::VertexSet& GetSetA() const {
        return set_A_;
    }

    const Graph::VertexSet& GetSetB() const {
        return set_B_;
    }

    const int GetDeltaAfterMove(Graph::Vertex v) { // Delta of Max Cut after moving vertex v from one part to other
        int delta = 0;
        if (set_A_.find(v) != set_A_.end()) {
            for (const auto& u : graph_.AdjacentVertices(v)) {
                if (set_A_.find(u) != set_A_.end()) {
                    delta++;
                } else {
                    delta--;
                }
            }
        } else if (set_B_.find(v) != set_B_.end()) {
            for (const auto& u : graph_.AdjacentVertices(v)) {
                if (set_B_.find(u) != set_B_.end()) {
                    delta++;
                } else {
                    delta--;
                }
            }
        }
        return delta;
    }

    void MoveVertex(Graph::Vertex v) {
        if (set_A_.find(v) != set_A_.end()) {
            set_A_.erase(v);
            set_B_.insert(v);
        } else if (set_B_.find(v) != set_B_.end()) {
            set_A_.insert(v);
            set_B_.erase(v);
        }
        return;
    }

    size_t GetSizeOfCut() const {
        size_t cut_size = 0;
        for (const auto& v : set_A_) {
            const auto& neighbours = graph_.AdjacentVertices(v);
            for (const auto& u : set_B_) {
                if (neighbours.find(u) != neighbours.end()) {
                    cut_size++;
                }
            }
        }
        return cut_size;
    }

    const Graph& GetGraph() const {
        return graph_;
    }

  private:

    const Graph& graph_;
    Graph::VertexSet set_A_, set_B_;
};


class MaxCutSolver {
  public:
    virtual VertexCut Solve(const Graph& graph) const = 0;
    virtual ~MaxCutSolver() = default;
};

class GreedySolver final: public MaxCutSolver {
    // TODO: insert implementation
};

class  AnnealingSolver final: public MaxCutSolver {
    // TODO: insert implementation
};

Graph RandomGraph(size_t size, double edge_probability) {
    Graph graph;
    for (Graph::Vertex v = 1; v <= size; ++v) {
        graph.AddVertex(v);
    }
    for (Graph::Vertex v = 1; v <= size; ++v) {
        for (Graph::Vertex u = v + 1; u <= size; ++u) {
            if (double(rand()) / RAND_MAX <= edge_probability) {
                graph.AddEdge(v, u);
            }
        }
    }
    return graph;
}

Graph StarGraph(size_t size) {
    Graph graph;
    for (Graph::Vertex v = 2; v <= size; ++v) {
        graph.AddEdge(1, v);
    }
    return graph;
}

Graph BipartiteGraph(size_t size, double edge_probability) {
    Graph graph;
    Graph::VertexSet left;
    Graph::VertexSet right;
    for (Graph::Vertex v = 1; v <= size; ++v) {
        graph.AddVertex(v);
        if (double(rand()) / RAND_MAX <= 0.5) {
            left.insert(v);
        } else {
            right.insert(v);
        }
    }

    for (const auto& v : left) {
        for (const auto& u : right) {
            if (double(rand()) / RAND_MAX <= edge_probability) {
                graph.AddEdge(v, u);
            }
        }
    }


    return graph;
}

int InitRandSeed(int argc, const char* argv[]) {
    int rand_seed;
    if (argc >= 2) {
        rand_seed = atoi(argv[1]);
    } else {
        rand_seed = time(nullptr);
    }
    srand(rand_seed);
    return rand_seed;
}

void TrySolver(const MaxCutSolver& solver, const std::vector<Graph>& graphs) {
    int result = 0;
    for (const auto& graph : graphs) {
        const auto vertex_cut = solver.Solve(graph);
        auto cost = vertex_cut.GetSizeOfCut();
        result += cost;
    }

    std::cout << "Result: " << result << std::endl;
}

int main(int argc, const char* argv[]) {
    std::cout << "Using rand seed: " << InitRandSeed(argc, argv) << "\n";

    std::vector<Graph> graphs;
    graphs.push_back(RandomGraph(8, 2.));
    GreedySolver greedy_solver ;
    AnnealingSolver annealing_solver(1);
    TrySolver(greedy_solver, graphs);
    TrySolver(annealing_solver, graphs);

    return 0;
}