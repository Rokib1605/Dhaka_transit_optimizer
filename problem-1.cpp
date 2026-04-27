#include <bits/stdc++.h>
using namespace std;

struct Vertex {
    pair<double,double> lonLat;                 
    vector<pair<int,double>> neighbors;         
    double distFromSrc = numeric_limits<double>::infinity();
    int prev = -1;

    Vertex() = default;
    Vertex(pair<double,double> ll) : lonLat(ll) {}
};

static inline double deg2rad(double deg) { return deg * (acos(-1.0) / 180.0); }

double haversine_km(const pair<double,double>& a, const pair<double,double>& b) {
    const double R = 6371.0; 
    double lon1 = deg2rad(a.first);
    double lat1 = deg2rad(a.second);
    double lon2 = deg2rad(b.first);
    double lat2 = deg2rad(b.second);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double s = sin(dlat/2.0);
    double t = sin(dlon/2.0);
    double A = s*s + cos(lat1) * cos(lat2) * t*t;
    double C = 2.0 * atan2(sqrt(A), sqrt(1.0 - A));
    return R * C;
}

int getOrCreateVertex(map<pair<double,double>,int> &coordToId, vector<Vertex> &graph, const pair<double,double> &coord) {
    auto it = coordToId.find(coord);
    if (it != coordToId.end()) return it->second;

    int newId = (int)graph.size();
    graph.emplace_back(Vertex(coord));
    coordToId[coord] = newId;
    return newId;
}

int findNearestVertex(const vector<Vertex>& graph, const pair<double,double>& coord, int excludeId = -1) {
    int bestId = -1;
    double bestDist = numeric_limits<double>::infinity();
    for (int i = 1; i < (int)graph.size(); ++i) {
        if (i == excludeId) continue;
        double d = haversine_km(coord, graph[i].lonLat);
        if (d < bestDist) {
            bestDist = d;
            bestId = i;
        }
    }
    return bestId;
}

void run_dijkstra(int srcId, vector<Vertex> &graph) {
    for (int i = 1; i < (int)graph.size(); ++i) {
        graph[i].distFromSrc = numeric_limits<double>::infinity();
        graph[i].prev = -1;
    }
    if (srcId <= 0 || srcId >= (int)graph.size()) return;
    graph[srcId].distFromSrc = 0.0;

    set<pair<double,int>> q;
    for (int i = 1; i < (int)graph.size(); ++i) q.insert({graph[i].distFromSrc, i});

    while (!q.empty()) {
        auto it = q.begin();
        int v = it->second;
        q.erase(it);

        if (graph[v].distFromSrc == numeric_limits<double>::infinity()) break;

        for (auto &edge : graph[v].neighbors) {
            int u = edge.first;
            double w = edge.second;
            double candidate = graph[v].distFromSrc + w;
            if (graph[u].distFromSrc > candidate) {
                auto found = q.find({graph[u].distFromSrc, u});
                if (found != q.end()) q.erase(found);
                graph[u].distFromSrc = candidate;
                graph[u].prev = v;
                q.insert({graph[u].distFromSrc, u});
            }
        }
    }
}

vector<pair<double,double>> parseCoordinatePairsFromCsvLine(const string &line) {
    vector<pair<double,double>> out;
    if (line.empty()) return out;

    stringstream ss(line);
    string token;
    if (!getline(ss, token, ',')) return out;

    vector<string> toks;
    while (getline(ss, token, ',')) toks.push_back(token);

    for (size_t i = 0; i + 1 < toks.size(); i += 2) {
        const string &lonStr = toks[i];
        const string &latStr = toks[i+1];
        if (lonStr.empty() || latStr.empty()) continue;
        try {
            double lon = stod(lonStr);
            double lat = stod(latStr);
            out.emplace_back(lon, lat);
        } catch (...) {
            continue;
        }
    }
    return out;
}

void write_kml_file(const string &filename, const vector<int> &path, const vector<Vertex> &graph) {
    ofstream kml(filename);
    if (!kml.is_open()) {
        cerr << "Could not write KML file: " << filename << '\n';
        return;
    }

    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    kml << "  <Document>\n";
    kml << "    <name>Shortest Path</name>\n";
    kml << "    <Placemark>\n";
    kml << "      <name>Route</name>\n";
    kml << "      <Style>\n";
    kml << "        <LineStyle>\n";
    kml << "          <color>ff0000ff</color>\n";
    kml << "          <width>4</width>\n";
    kml << "        </LineStyle>\n";
    kml << "      </Style>\n";
    kml << "      <LineString>\n";
    kml << "        <tessellate>1</tessellate>\n";
    kml << "        <coordinates>\n";

    for (int id : path) {
        double lon = graph[id].lonLat.first;
        double lat = graph[id].lonLat.second;
        kml << "          " << lon << "," << lat << ",0\n";
    }

    kml << "        </coordinates>\n";
    kml << "      </LineString>\n";
    kml << "    </Placemark>\n";
    kml << "  </Document>\n";
    kml << "</kml>\n";

    kml.close();
}

string modeToString(int mode) {
    if (mode == 1) return "Walk";
    if (mode == 2) return "Car";
    return "Unknown";
}

void print_plain_route(const pair<double,double>& srcCoord,
                       const pair<double,double>& dstCoord,
                       double totalDistance,
                       const vector<int>& path,
                       const vector<Vertex>& graph,
                       const map<pair<int,int>,int>& edgeMode) {
    cout << fixed << setprecision(6);
    cout << "SOURCE:      (" << srcCoord.first << ", " << srcCoord.second << ")\n";
    cout << "DESTINATION: (" << dstCoord.first << ", " << dstCoord.second << ")\n";
    cout << "TOTAL DISTANCE: " << totalDistance << " km\n";
    cout << "\n";
    cout << "ROUTE SEGMENTS:\n";

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int u = path[i], v = path[i+1];
        double d = haversine_km(graph[u].lonLat, graph[v].lonLat);
        int modeInt = 0;
        auto it = edgeMode.find({u,v});
        if (it != edgeMode.end()) modeInt = it->second;
        string modeStr = modeToString(modeInt);

        cout << "Segment " << (i+1) << ": From (" 
             << graph[u].lonLat.first << ", " << graph[u].lonLat.second << ")"
             << " -> To (" << graph[v].lonLat.first << ", " << graph[v].lonLat.second << ")"
             << " | Mode: " << modeStr
             << " | Distance: " << d << " km\n";
    }

    cout << "\n";
    cout << "KML: Problem-1.kml\n";
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    vector<Vertex> graph;
    graph.emplace_back(); 

    map<pair<double,double>, int> coordToId;
    map<pair<int,int>, int> edgeMode; 

    ifstream roadmap("./Roadmap-Dhaka.csv");
    if (!roadmap.is_open()) {
        cerr << "Cant open the dataset of roadmap\n";
        return 1;
    }

    string line;
    while (getline(roadmap, line)) {
        auto coords = parseCoordinatePairsFromCsvLine(line);
        if (coords.size() < 2) continue;

        for (size_t i = 0; i + 1 < coords.size(); ++i) {
            int u = getOrCreateVertex(coordToId, graph, coords[i]);
            int v = getOrCreateVertex(coordToId, graph, coords[i+1]);
            double d = haversine_km(coords[i], coords[i+1]);

            graph[u].neighbors.emplace_back(v, d);
            graph[v].neighbors.emplace_back(u, d);

            edgeMode[{u,v}] = 2;
            edgeMode[{v,u}] = 2;
        }
    }
    roadmap.close();

    pair<double,double> srcCoord, dstCoord;
    if (!(cin >> srcCoord.first >> srcCoord.second >> dstCoord.first >> dstCoord.second)) {
        cerr << "Failed to read source/destination coordinates from stdin\n";
        return 1;
    }

    int srcId, dstId;

    auto itSrc = coordToId.find(srcCoord);
    if (itSrc == coordToId.end()) {
        int nearest = findNearestVertex(graph, srcCoord, -1);
        if (nearest == -1) {
            cerr << "No vertices in graph to connect source to.\n";
            return 1;
        }
        double d = haversine_km(srcCoord, graph[nearest].lonLat);
        srcId = (int)graph.size();
        coordToId[srcCoord] = srcId;
        graph.emplace_back(Vertex(srcCoord));
        graph[srcId].neighbors.emplace_back(nearest, d);
        graph[nearest].neighbors.emplace_back(srcId, d);
        edgeMode[{srcId, nearest}] = 1;
        edgeMode[{nearest, srcId}] = 1;
    } else {
        srcId = itSrc->second;
    }

    auto itDst = coordToId.find(dstCoord);
    if (itDst == coordToId.end()) {
        int nearest = findNearestVertex(graph, dstCoord, srcId); 
        if (nearest == -1) {
            cerr << "No vertices in graph to connect destination to.\n";
            return 1;
        }
        double d = haversine_km(dstCoord, graph[nearest].lonLat);
        dstId = (int)graph.size();
        coordToId[dstCoord] = dstId;
        graph.emplace_back(Vertex(dstCoord));
        graph[dstId].neighbors.emplace_back(nearest, d);
        graph[nearest].neighbors.emplace_back(dstId, d);
        edgeMode[{dstId, nearest}] = 1;
        edgeMode[{nearest, dstId}] = 1;
    } else {
        dstId = itDst->second;
    }

    run_dijkstra(srcId, graph);

    if (graph[dstId].distFromSrc == numeric_limits<double>::infinity()) {
        cout << "NO path\n\n";
        return 0;
    }

    vector<int> path;
    int cur = dstId;
    while (cur != -1) {
        path.push_back(cur);
        cur = graph[cur].prev;
    }
    reverse(path.begin(), path.end());

    write_kml_file("Problem-1.kml", path, graph);

    print_plain_route(srcCoord, dstCoord, graph[dstId].distFromSrc, path, graph, edgeMode);

    return 0;
}
