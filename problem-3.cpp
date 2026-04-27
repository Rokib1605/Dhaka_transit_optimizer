#include <bits/stdc++.h>
using namespace std;

struct Vertex {
    pair<double,double> lonlat;              
    vector<pair<int,double>> adj;            
    double cost = numeric_limits<double>::infinity();
    int prev = -1;
    Vertex() = default;
    Vertex(pair<double,double> p): lonlat(p) {}
};

static inline double deg2rad(double deg) { return deg * (acos(-1.0) / 180.0); }

double haversine_km(const pair<double,double>& a, const pair<double,double>& b) {
    const double R = 6371.0;
    double lon1 = deg2rad(a.first), lat1 = deg2rad(a.second);
    double lon2 = deg2rad(b.first), lat2 = deg2rad(b.second);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double s = sin(dlat/2.0);
    double t = sin(dlon/2.0);
    double A = s*s + cos(lat1) * cos(lat2) * t*t;
    double C = 2.0 * atan2(sqrt(A), sqrt(1.0 - A));
    return R * C;
}

string trim(const string &s) {
    const string WHITESPACE = " \n\r\t\f\v";
    size_t start = s.find_first_not_of(WHITESPACE);
    if (start == string::npos) return "";
    size_t end = s.find_last_not_of(WHITESPACE);
    return s.substr(start, end - start + 1);
}

vector<pair<double,double>> parse_lonlat_pairs(const string &line) {
    vector<pair<double,double>> out;
    if (line.empty()) return out;
    stringstream ss(line);
    string token;
    if (!getline(ss, token, ',')) return out;
    vector<string> toks;
    while (getline(ss, token, ',')) toks.push_back(trim(token));
    for (size_t i = 0; i + 1 < toks.size(); i += 2) {
        if (toks[i].empty() || toks[i+1].empty()) continue;
        try {
            double lon = stod(toks[i]);
            double lat = stod(toks[i+1]);
            out.emplace_back(lon, lat);
        } catch (...) {
            continue;
        }
    }
    return out;
}

int find_or_create_vertex(map<pair<double,double>,int> &coordToId, vector<Vertex> &nodes, const pair<double,double> &coord) {
    auto it = coordToId.find(coord);
    if (it != coordToId.end()) return it->second;
    int id = (int)nodes.size();
    nodes.emplace_back(Vertex(coord));
    coordToId[coord] = id;
    return id;
}

void build_graph_from_csv(const string &filename,
                          vector<Vertex> &nodes,
                          map<pair<double,double>,int> &coordToId,
                          map<pair<int,int>,int> &edgeMode,
                          int mode)
{
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Cant open the dataset of map - " << filename << '\n';
        return;
    }
    int costPerKm = 7;
    if (mode == 2) costPerKm = 20;
    else if (mode == 3) costPerKm = 5;
    else if (mode == 4) costPerKm = 7;
    else if (mode == 5) costPerKm = 7;

    string line;
    while (getline(fin, line)) {
        auto pts = parse_lonlat_pairs(line);
        if (pts.size() < 2) continue;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            int u = find_or_create_vertex(coordToId, nodes, pts[i]);
            int v = find_or_create_vertex(coordToId, nodes, pts[i+1]);
            double d_km = haversine_km(pts[i], pts[i+1]);
            double cost = d_km * costPerKm;
            nodes[u].adj.emplace_back(v, cost);
            nodes[v].adj.emplace_back(u, cost);
            edgeMode[{u,v}] = mode;
            edgeMode[{v,u}] = mode;
        }
    }
    fin.close();
}

void dijkstra_shortest_cost(int src, vector<Vertex> &nodes) {
    for (int i = 1; i < (int)nodes.size(); ++i) {
        nodes[i].cost = numeric_limits<double>::infinity();
        nodes[i].prev = -1;
    }
    if (src <= 0 || src >= (int)nodes.size()) return;
    nodes[src].cost = 0.0;
    set<pair<double,int>> st;
    for (int i = 1; i < (int)nodes.size(); ++i) st.insert({nodes[i].cost, i});
    while (!st.empty()) {
        auto it = st.begin();
        int v = it->second;
        st.erase(it);
        if (nodes[v].cost == numeric_limits<double>::infinity()) break;
        for (auto &e : nodes[v].adj) {
            int u = e.first;
            double w = e.second;
            double cand = nodes[v].cost + w;
            if (nodes[u].cost > cand) {
                auto found = st.find({nodes[u].cost, u});
                if (found != st.end()) st.erase(found);
                nodes[u].cost = cand;
                nodes[u].prev = v;
                st.insert({nodes[u].cost, u});
            }
        }
    }
}

void write_kml(const string &filename, const vector<int> &path, const vector<Vertex> &nodes) {
    ofstream kml(filename);
    if (!kml.is_open()) {
        cerr << "Could not write KML file\n";
        return;
    }
    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    kml << "<Document>\n";
    kml << "<name>Cheapest Path</name>\n";
    kml << "<Placemark>\n";
    kml << "<name>Route</name>\n";
    kml << "<Style>\n";
    kml << "<LineStyle>\n";
    kml << "<color>ff0000ff</color>\n";
    kml << "<width>4</width>\n";
    kml << "</LineStyle>\n";
    kml << "</Style>\n";
    kml << "<LineString>\n";
    kml << "<tessellate>1</tessellate>\n";
    kml << "<coordinates>\n";
    for (int id : path) {
        double lon = nodes[id].lonlat.first;
        double lat = nodes[id].lonlat.second;
        kml << lon << "," << lat << ",0\n";
    }
    kml << "</coordinates>\n";
    kml << "</LineString>\n";
    kml << "</Placemark>\n";
    kml << "</Document>\n";
    kml << "</kml>\n";
    kml.close();
}

string mode_name(int mode) {
    if (mode == 1) return "Walk";
    if (mode == 2) return "Car";
    if (mode == 3) return "Metro";
    if (mode == 4) return "Uttara Bus";
    if (mode == 5) return "Bikolpo Bus";
    return "Unknown";
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    vector<Vertex> nodes;
    nodes.emplace_back(); 

    map<pair<double,double>,int> coordToId;
    map<pair<int,int>,int> edgeMode; 

    build_graph_from_csv("Roadmap-Dhaka.csv", nodes, coordToId, edgeMode, 2);
    build_graph_from_csv("Routemap-DhakaMetroRail.csv", nodes, coordToId, edgeMode, 3);
    build_graph_from_csv("Routemap-UttaraBus.csv", nodes, coordToId, edgeMode, 4);
    build_graph_from_csv("Routemap-BikolpoBus.csv", nodes, coordToId, edgeMode, 5);

    pair<double,double> src, dst;
    if (!(cin >> src.first >> src.second >> dst.first >> dst.second)) {
        cerr << "Failed to read source/destination coordinates from stdin\n";
        return 1;
    }

    cout << fixed << setprecision(6);
    cout << "Source Longitude = " << src.first << "\n";
    cout << "Source Latitude = " << src.second << "\n";
    cout << "Destination Longitude = " << dst.first << "\n";
    cout << "Destination Latitude = " << dst.second << "\n";

    int srcId, dstId;
    auto itSrc = coordToId.find(src);
    if (itSrc == coordToId.end()) {
        int nearest = -1;
        double best = numeric_limits<double>::infinity();
        for (int i = 1; i < (int)nodes.size(); ++i) {
            double d = haversine_km(src, nodes[i].lonlat);
            if (d < best) { best = d; nearest = i; }
        }
        if (nearest == -1) { cerr << "Graph empty - cannot connect source\n"; return 1; }
        srcId = (int)nodes.size();
        coordToId[src] = srcId;
        nodes.emplace_back(Vertex(src));
        nodes[srcId].adj.emplace_back(nearest, 0.0);
        nodes[nearest].adj.emplace_back(srcId, 0.0);
        edgeMode[{srcId, nearest}] = 1;
        edgeMode[{nearest, srcId}] = 1;
    } else srcId = itSrc->second;

    auto itDst = coordToId.find(dst);
    if (itDst == coordToId.end()) {
        int nearest = -1;
        double best = numeric_limits<double>::infinity();
        for (int i = 1; i < (int)nodes.size(); ++i) {
            if (i == srcId) continue;
            double d = haversine_km(dst, nodes[i].lonlat);
            if (d < best) { best = d; nearest = i; }
        }
        if (nearest == -1) { cerr << "Graph empty - cannot connect destination\n"; return 1; }
        dstId = (int)nodes.size();
        coordToId[dst] = dstId;
        nodes.emplace_back(Vertex(dst));
        nodes[dstId].adj.emplace_back(nearest, 0.0);
        nodes[nearest].adj.emplace_back(dstId, 0.0);
        edgeMode[{dstId, nearest}] = 1;
        edgeMode[{nearest, dstId}] = 1;
    } else dstId = itDst->second;

    dijkstra_shortest_cost(srcId, nodes);

    if (nodes[dstId].cost == numeric_limits<double>::infinity()) {
        cout << "NO path\n";
        return 0;
    }

    cout << "Cheapest Cost = " << nodes[dstId].cost << " (Tk)\n";

    vector<int> path;
    int cur = dstId;
    while (cur != -1) {
        path.push_back(cur);
        cur = nodes[cur].prev;
    }
    reverse(path.begin(), path.end());

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int u = path[i], v = path[i+1];
        double dist_km = haversine_km(nodes[u].lonlat, nodes[v].lonlat);
        int mode = 1;
        auto it = edgeMode.find({u,v});
        if (it != edgeMode.end()) mode = it->second;
        cout << "Segment " << (i+1) << ": From ("
             << nodes[u].lonlat.first << ", " << nodes[u].lonlat.second << ")"
             << " -> To (" << nodes[v].lonlat.first << ", " << nodes[v].lonlat.second << ")"
             << " | Mode: " << mode_name(mode)
             << " | Distance: " << fixed << setprecision(6) << dist_km << " km\n";
    }

    write_kml("Problem-3.kml", path, nodes);
    cout << "KML written to Problem-3.kml\n";
    return 0;
}
