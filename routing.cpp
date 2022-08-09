/*---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---*/

#include <bits/stdc++.h>
using namespace std;

#define LOCAL
#define rep(i, a, b)	for(int i = a; i < (b); ++i)
#define rrep(a, b, c)	for(int a = (b); a > c; --a)
#define each(a, b)		for(auto& a : b)

#define sz(x)       (int)(x).size()
#define all(a)      (a).begin(),(a).end()
#define rall(a)     (a).rbegin(), (a).rend()

#define lb lower_bound
#define ub upper_bound
#define pb push_back
#define eb emplace_back
#define mp make_pair
#define f first
#define s second
#define V vector
#define ar array
#define pi pair<int, int>
#define vi vector<int>
#define pll pair<ll, ll>
#define pct(x) __builtin_popcount(x)
#define rsz resize
#define bk back()

constexpr int log2(int x) { return x == 0 ? 0 : 31-__builtin_clz(x); } // floor(log2(x))
mt19937 rng((uint32_t)chrono::steady_clock::now().time_since_epoch().count());

template <class T> bool umin(T& a, const T& b){return b<a?a=b, 1:0;}
template <class T> bool umax(T& a, const T& b){return a<b?a=b, 1:0;}

using ll = long long;
using ld = long double;
using str = string;

const int inf = (int)1e9 + 5;
const ll infl = (ll)1e18 + 5;
const int N = 100;
const int MX = 1e6;
const int M = 10;

void __print(int x) {cout << x;}
void __print(long x) {cout << x;}
void __print(long long x) {cout << x;}
void __print(unsigned x) {cout << x;}
void __print(unsigned long x) {cout << x;}
void __print(unsigned long long x) {cout << x;}
void __print(float x) {cout << x;}
void __print(double x) {cout << x;}
void __print(long double x) {cout << x;}
void __print(char x) {cout << '\'' << x << '\'';}
void __print(const char *x) {cout << '\"' << x << '\"';}
void __print(const string &x) {cout << '\"' << x << '\"';}
void __print(bool x) {cout << (x ? "true" : "false");}
 
template<typename T, typename V>
void __print(const pair<T, V> &x) {cout << '{'; __print(x.first); cout << ", "; __print(x.second); cout << '}';}
template<typename T>
void __print(const T &x) {int f = 0; cout << '{'; for (auto &i: x) cout << (f++ ? ", " : ""), __print(i); cout << "}";}
void _print() {cout << "]\n";}
template <typename T, typename... V>
void _print(T t, V... v) {__print(t); if (sizeof...(v)) cout << ", "; _print(v...);}
#ifdef LOCAL
#define dbg(x...) cout <<" [" << #x << "] = ["; _print(x);
#else
#define dbg(x...)
#endif

/*---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---XXX---*/
str s;
int layers, tracks, buses, obstacles, tmp;

map<str, int> mlti; // map from layer name to index

struct P{
	int x, y;

	P(int x, int y) : x(x), y(y) {}
	P() {}

	void inp(){
		char c;
		cin >> c >> x >> y >> c;
	}

	bool operator < (const P& b) const {
		return (x == b.x ? y < b.y : x < b.x);
	}

	bool operator > (const P& b) const {
		return (x == b.x ? y > b.y : x > b.x);
	}

	bool operator == (const P& b) const {
		return x == b.x && y == b.y;
	}

	friend ostream& operator << (ostream &stream, const P &p){
		stream << '(' << p.x << ", " << p.y << ") ";
		return stream;
	}
};

struct Rect{
	P a, b;
	int lay;

	void inp(){
		cin >> s;
		lay = mlti[s];
		a.inp(); b.inp();
	}

	ar<int, 3> giveinarmid(){
		return {(a.x+b.x)/2, (a.y+b.y)/2, lay};
	}

	bool Pinside(P c, int l = -1) const{
		if((l == -1 || l == lay) && c.x <= b.x && c.x >= a.x && c.y <= b.y && c.y >= a.y) return 1;
		return 0;
	}

	P mid(){
		return {(a.x+b.x)/2, (a.y+b.y)/2};
	}

	friend ostream& operator << (ostream &stream, const Rect &r) {
		stream << r.a << ',' << r.b << '\n';
		return stream;
	}
};

struct Track{
	Rect r;
	int width;

	void inp(){
		r.inp();
		cin >> width;
	}
};


struct Layer{
	static int total_layer;
	bool dir;
	int spconst;
	str name;

	void inp(){
		cin >> name;
		mlti[name] = total_layer++;
		cin >> s;
		if(s == "vertical") dir = 1;
		cin >> spconst;
	}
};

int Layer::total_layer = 0;

struct BIT{
	V<Rect> v;
	str name;

	void inp(){
		v.emplace_back();
		v.bk.inp();
	}
};

struct BUS{
	int np, npg;
	vi width;
	V<BIT> B;

	void inp(){
		cin >> s >> s >> np >> npg;
		assert(npg == 2); // assuming npg == 2
		// WIDTH
		cin >> s >> tmp;
		width.rsz(tmp);
		rep(i, 0, tmp){
			cin >> width[i];
		}
		cin >> s;

		B.rsz(np);
		rep(i, 0, np){
			cin >> s >> B[i].name;
			rep(j, 0, npg) B[i].inp();
			cin >> s;
		}
	}
};

P lowl, upr; // Design Boundary lower left, upper right
Layer lay[N];
V<Track> admin_track[N];
BUS bus[N];
V<Rect> obs[N];

void take_input(){
	int x;
	
	cin >> s >> x; // RUNTIME
	cin >> s >> x; // ALPHA
	cin >> s >> x; // BETA
	cin >> s >> x; // GAMMA	
	cin >> s >> x; // DELTA
	cin >> s >> x; // EPSILON

	// DESIGN BOUNDARY
	cin >> s;
	lowl.inp(); upr.inp();

	// LAYERS
	cin >> s >> layers;
	rep(i, 0, layers){
		lay[i].inp();
	}
	cin >> s;

	// TRACKS
	cin >> s >> tracks;
	Track tm;
	rep(i, 0, tracks){
		tm.inp();
		admin_track[tm.r.lay].eb(tm);
	}
	cin >> s;

	// BUSES
	cin >> s >> buses;
	rep(i, 0, buses){
		bus[i].inp();
		cin >> s;
	}
	cin >> s;

	// OBSTACLES
	cin >> s >> obstacles;
	Rect r;
	rep(i, 0, obstacles){
		r.inp();
		obs[r.lay].eb(r);
	}
	cin >> s;

	cout << "INPUT TAKING FINISH" << endl;
}

BUS cur;

map<P, int> ctn[N]; // coordinates to node
V<ar<int, 3>> ntc; // nodes to coordinates
int nn; // # of nodes
V<vi> g; // graph

P interpt(const Track &a, const Track &b){ // intersection point between tracks
	Rect const &x = a.r, &y = b.r;
	P ans = {-1, -1}, t;
	// if(abs(x.lay - y.lay) == 1){
		if(lay[x.lay].dir == 0) t = {y.a.x, x.a.y};
		else t = {x.a.x, y.a.y};
		if(x.Pinside(t) && y.Pinside(t)) ans = t; 
	// }
	return ans;
}

V<Track> track[N];

vi Hsh[N];
void cleantracks(){
	// OBSTACLES IMPLEMTATION ON TRACKS
	// 1 OBSTACLE BETWEEN A TRACK CUT IT INTO TWO TRACKS


	rep(i, 0, layers){
		if(lay[i].dir){
			sort(all(obs[i]), [](Rect const &r1, Rect const &r2){
				if(r1.a.y == r2.a.y) r1.a.x < r2.a.x;
				return r1.a.y < r2.a.y;
			});

			for(Track x : admin_track[i]){
				each(r, obs[i]){
					if(x.r.a.y > x.r.b.y) break;
					if(r.a.x <= x.r.a.x && x.r.a.x <= r.b.x && r.b.y >= x.r.a.y && r.a.y <= x.r.b.y){
						if(r.a.y <= x.r.a.y){
							x.r.a.y = r.b.y + 1;
						}else if(r.b.y >= x.r.b.y){
							x.r.b.y = r.a.y - 1;
						}else{
							Track y = x;
							y.r.b.y = r.a.y - 1;
							track[i].eb(y);
							// cout << y.r << '\n';
							x.r.a.y = r.b.y + 1;
						}
					}
				}
				if(x.r.a.y < x.r.b.y){
					track[i].eb(x);
					// cout << x.r << '\n';
				}
			}

		}else{
			// cout << "HARAM" << '\n';
			sort(all(obs[i]), [](Rect const &r1, Rect const &r2){
				if(r1.a.x == r2.a.x) r1.a.y < r2.a.y;
				return r1.a.x < r2.a.x;
			});

			for(Track x : admin_track[i]){
				// cout << "ADMIN => " <<  x.r << '\n';
				each(r, obs[i]){
					if(x.r.a.x > x.r.b.x) break;
					if(r.a.y <= x.r.a.y && x.r.a.y <= r.b.y && r.b.x >= x.r.a.x && r.a.x <= x.r.b.x){
						if(r.a.x <= x.r.a.x){
							x.r.a.x = r.b.x + 1;
						}else if(r.b.x >= x.r.b.x){
							x.r.b.x = r.a.x - 1;
						}else{
							Track y = x;
							y.r.b.x = r.a.x - 1;
							track[i].eb(y);
							// cout << y.r << '\n';
							x.r.a.x = r.b.x + 1;
						}
					}
				}
				if(x.r.a.x < x.r.b.x){
					track[i].eb(x);
					// cout << x.r << '\n';
				}
			}
		}

	}

	rep(i, 0, layers){
		if(lay[i].dir == 0){
			sort(all(track[i]), [](Track const &a, Track const &b){
				if(a.r.a.y == b.r.a.y) return a.r.a.x < b.r.a.x;
				return a.r.a.y < b.r.a.y;
			});
		}else{
			sort(all(track[i]), [](Track const &a, Track const &b){
				if(a.r.a.x == b.r.a.x) return a.r.a.y < b.r.a.y;
				return a.r.a.x < b.r.a.x;
			});
		}
		// dbg(i)
		each(x, track[i]){
			// cout << x.r;
			int tm = (lay[i].dir ? x.r.a.x : x.r.a.y);
			if(Hsh[i].empty() || Hsh[i].bk^tm) Hsh[i].eb(tm);
		}
	}
}

void generategraph(){
	// initialsing variables...
	nn = 1; g.rsz(1); ntc.rsz(1);
	rep(i, 0, layers) ctn[i].clear();
	cleantracks();
	int cnt = 0, xxx = 1e6;
	P p;

	rep(i, 0, layers){
		bool dir = lay[i].dir;
		rep(j, i-1, i+2){
			if(j == i || j == -1 || j == layers) continue;
			P t = {-1, -1}; int lst = -1;
			each(x, track[i]){
				bool ok = (lst == -1 ? 0 : x.r.Pinside(t));

				each(y, track[j]){
					p = interpt(x, y);
					if(p.x == -1) continue;

					// cout << i << " => " << x.r;
					// cout << j << " => " << y.r << p << '\n';
					
					cnt++;
					if(cnt == xxx) cout << cnt << endl, xxx += 1e6 ;

					auto &it = ctn[i][p];
					if(!it) it = nn++, g.eb(), ntc.pb({p.x, p.y, i});

					auto &it1 = ctn[j][p];
					if(!it1) it1 = nn++, g.eb(), ntc.pb({p.x, p.y, j});

					if(!ok || ((dir && p.y > t.y) || (dir == 0 && p.x > t.x))){
						g[it].eb(it1);
						if(!umax(ok, (bool)1)){
							g[lst].eb(it);
							g[it].eb(lst);
						}
						t = p; lst = it;
					}
				}

				p = x.r.b;
				if(p == t) continue;
				auto &it = ctn[i][p];
				if(!it) it = nn++, g.eb(), ntc.pb({p.x, p.y, i});

				if(!ok || ((dir && p.y > t.y) || (dir == 0 && p.x > t.x))){
					if(!umax(ok, (bool)1)){
						g[lst].eb(it);
						g[it].eb(lst);
					}
					t = p; lst = it;
				}
			}
		}
	}



	each(x, cur.B){
		each(r, x.v){
			p = r.mid();
			if(ctn[r.lay].find(p) != ctn[r.lay].end()) continue;

			bool dir = lay[r.lay].dir;

			int n1 = -1, n2 = -1;
			if(dir == 0){
				P p1 = {-1, p.y}, p2 = {inf, p.y};
				each(z, ctn[r.lay]){
					if(p.y == z.f.y){
						if(p.x <= z.f.x){
							if(umin(p2.x, z.f.x)) n2 = z.s;
						}
						if(p.x >= z.f.x){
							if(umax(p1.x, z.f.x)) n1 = z.s;
						}
					}
				}
			}else{
				P p1 = {p.x, -1}, p2 = {p.x, inf};
				each(z, ctn[r.lay]){
					if(p.x == z.f.x){
						if(p.y <= z.f.y){
							if(umin(p2.y, z.f.y)) n2 = z.s;
						}
						if(p.y >= z.f.y){
							if(umax(p1.y, z.f.y)) n1 = z.s;
						}
					}
				}
			}
			auto &it = ctn[r.lay][p];
			assert(it == 0);
			it = nn++, g.eb(); ntc.pb({p.x, p.y, r.lay});

			for(int q : {n1, n2}){
				if(q^-1){
					g[it].eb(q);
					g[q].eb(it);
				}
			}
		}
	}

	each(x, g){
		sort(all(x));
		x.erase(unique(all(x)), x.end());		
	}

	// rep(i, 1, nn){
	// 	dbg(ntc[i]);
	// 	dbg(g[i]);
	// }

	cout << "GRAPH MAKING FINISH" << endl;
}


int heuristic(int x, int y){
	int z = 0;
	ar<int, 3> &a = ntc[x], &b = ntc[y];
	rep(i, 0, 3){
		z += abs(a[i] - b[i]);
	}
	return z;
}

// V<bool> ali;
// int xq = 0;
// void dfs(int s, int p){
// 	if(ali[s]) return;
// 	xq++;
// 	ali[s] = 1;
// 	each(x, g[s]){
// 		dfs(x, s);
// 	}
// }


void Astar(BIT b, vi &v, V<ar<int, 3>> &path1){
	// f(n) = g(n) + h(n);

	int G[nn], par[nn]; // SEE OVERFLOW ISSUES
	bool vis[nn] = {0};
	memset(G, 0x3f, sizeof(G));

	int start = ctn[b.v[0].lay][b.v[0].mid()];
	int end = ctn[b.v[1].lay][b.v[1].mid()];
	priority_queue<P, V<P>, greater<P>> pq;
	pq.emplace(0, start);
	G[start] = 0;

	

	// ali.rsz(nn);
	// dfs(start, -1);
	// cout << "TRY => " <<  ali[end] << '\n';
	// dbg(xq);


	// memset(par, 0, sizeof(par));
	// dbg(ntc[start]);
	// dbg(ntc[end]);
	// dbg(g[end]);
	// cout << G[0] << ' ' << G[9] << '\n';

	while(!pq.empty()){
		int p = pq.top().y; pq.pop(); 
		int d = G[p];
		if(!umax(vis[p], (bool)1)) continue;
		if(p == end) break;
		// dbg(ntc[p], g[p]);
		each(y, g[p]){
			int kk = d + heuristic(p, y);
			// dbg(ntc[y], G[y], kk, G[p]);
			if(umin(G[y], d + heuristic(p, y))){
				par[y] = p;
				pq.emplace(G[y] + heuristic(y, end), y);
			}
		}
	}
	// dbg(xxx);
	// dbg(vis[end]);
	// cout << vis[] << '\n';

	// retrace path
	int t = end;
	vi path = {t};
	// V<ar<int, 3>> justdbg = {ntc[t]};
	while(t != start){
		// cout << t << endl;
		// dbg(ntc[t]);
		t = par[t];
		path.eb(t);
		// justdbg.eb(ntc[t]);
	}

	// cout << "TWX" << endl;

	reverse(all(path));
	// reverse(all(justdbg));
	// dbg(justdbg);
	int tmp = -1;
	each(x, path){
		if(ntc[x][2]^tmp) v.eb(tmp = ntc[x][2]), path1.eb(ntc[x]);
	}

	if(path1.bk != ntc[path.bk]) path1.eb(ntc[path.bk]);
	dbg(path1, v);
}

struct TMP{
	int f; vi v;
	V<ar<int, 3>> path;

	TMP(int f, vi v, V<ar<int, 3>> p) : f(f), v(v), path(p) {}
	bool operator < (const TMP &b) const{
		return f < b.f;
	}

	void print(){
		cout << f << ' ';
		dbg(v);
	}
};


V<TMP> TopologicalAnalysis(const V<vi> &Sg, const V<V<ar<int, 3>>> &path){
	map<vi, pi> m;

	// choose middle one if result is not correct...

	rep(i, 0, sz(Sg)){
		auto &x = Sg[i];
		if(m.find(x) == m.end()){
			m[x] = {1, i};
		}else{
			m[x].f++;
		}

	}

	V<TMP> v;
	each(x, m){
		v.eb(x.s.f, x.f, path[x.s.s]);
	}

	sort(all(v));
	return v;
}

int nfirst, nonfirst;


vi UTL(ar<int, 3> p, bool decis, bool both){
	int x = lb(all(Hsh[p[2]]), (lay[p[2]].dir ? p[0] : p[1])) - Hsh[p[2]].begin();
	int q = both ? nfirst : nonfirst;

	vi v;
	if(both || decis){
		rep(i, max(0, x-q+1), x){
			v.eb(Hsh[p[2]][i]);
		}
	}

	if(both) v.eb(Hsh[p[2]][x]);

	if(both || !decis){
		rep(i, x+1, min(x+q, sz(Hsh[p[2]]))){
			v.eb(Hsh[p[2]][i]);
		}
	}

	if(decis) reverse(all(v));

	return v;
}

	
struct Node{
	ar<int, 3> cord;
	V<P> v;
	int t;

	Node() {}
	Node(ar<int, 3> p, int t) : cord(p), t(t) {}

	void add(int i, int j){
		v.eb(i, t*j);
	}
};

V<ar<int, 3>> solvedag(const V<Node> &dag){
	vi dis(sz(dag), inf);
	vi par(sz(dag));

	priority_queue<P, V<P>, greater<P>> pq;
	pq.emplace(0, 0);
	dis[0] = 0;
	par[0] = -1;

	while(!pq.empty()){
		P a = pq.top(); pq.pop();
		if(a.x != dis[a.y]) continue;
		if(a.y + 1 == sz(dag)) break;
		each(x, dag[a.y].v){
			if(umin(dis[x.x], a.x + x.y)){
				pq.emplace(dis[x.x], x.x);
				par[x.x] = a.y;
			}
		}
	}

	// retrace path
	int t = sz(dag) - 1;
	V<ar<int, 3>> path;
	if(dis[t]^inf){
		while(t^-1){
			path.eb(dag[t].cord);
			t = par[t];
		}
		reverse(all(path));
		
		rep(i, 1, sz(path)){
			if(path[i-1][0] == path[i][0] && path[i-1][1] == path[i][1]){
				path.clear();
				break;
			}
		}
	}

	return path;
}

const int nmax = 1000;

V<V<ar<int, 3>>> solve(){
	nfirst = 6*cur.np;
	nonfirst = 3*cur.np;

	V<vi> Sg;
	V<V<ar<int, 3>>> path;	// Set of Routing Guides...
	rep(i, 0, cur.np){
		Sg.eb(); path.eb();
		Astar(cur.B[i], Sg.bk, path.bk);
	}

	cout << "PATH SEARCH THROUGH A* FINISH" << endl;

	V<TMP> PQg = TopologicalAnalysis(Sg, path);
	cout << "Priority Queue of Guides" << endl;
	each(x, PQg) x.print();
	V<V<ar<int, 3>>> ans;

	while(!PQg.empty()){
		vi gold = PQg.back().v;
		V<ar<int, 3>> p = PQg.bk.path, p1 = p;
		PQg.pop_back();
		

		// dbg(p)
		int nf = 0;
		while(nf < nmax){

			// Decision(G)
			vi Deci(sz(gold));
			each(x, Deci) x = rng()%2;


			rep(pin, 0, cur.np){
				ar<int, 3> start = cur.B[pin].v[0].giveinarmid();
				ar<int, 3> end = cur.B[pin].v[1].giveinarmid();
				V<Node> dag;

				// DAG CONSTRUCTION
				vi stk = {0};
				dag.eb(start, 1);

				rep(i, 1, sz(p)-1){
					vi utl;
					int pl = gold[i-1]; // previous layer
					if(i != sz(p)-2) utl = UTL(p[i], Deci[i], pin == 0);
					else{
						utl.eb(end[!lay[gold[i]].dir]);
					}
					
					vi newstk;
					each(pt, stk){
						if(lay[pl].dir){
							int x = dag[pt].cord[0];
							rep(j, 0, sz(utl)){
								int y = utl[j];
								auto it = ctn[pl].find({x, y});
								if(it == ctn[pl].end()) continue;
								newstk.eb(sz(dag));
								dag[pt].add(sz(dag), j+1);
								dag.pb({{x, y, gold[i]}, j+1});
							}
						}else{
							// y coordinate constant
							int y = dag[pt].cord[1];
							rep(j, 0, sz(utl)){
								int x = utl[j];
								auto it = ctn[pl].find({x, y});
								if(it == ctn[pl].end()) continue;
								newstk.eb(sz(dag));
								dag[pt].add(sz(dag), j+1);
								dag.pb({{x, y, gold[i]}, j+1});
							}
						}
					}
					swap(stk, newstk);
					// dbg(utl)
					// each(x, stk){
					// 	dbg(dag[x].cord);
					// }
				}

				each(pt, stk){
					dag[pt].add(sz(dag), 1);
				}
				dag.eb(end, 1);

				// SOLVE DAG
				p = solvedag(dag);

				if(p.empty()){ // NO SOLUTION
					++nf;
					ans.clear();
					break;
				}else{
					ans.eb(p);
				}
			}

			if(sz(ans) == cur.np) return ans;
			p = p1;
		}
	}

	return ans;
}

void print_path(const V<V<ar<int, 3>>> &path){
	cout << "PATHS" << '\n';
	assert(sz(path) == cur.np);
	
	rep(i, 0, cur.np){
		dbg(i, path[i]);
	}

	cout << "IN 2D COORDINATES" << '\n';
	int f = 0;
	rep(i, 0, sz(path)){
		cout << "[i, path] = " << i << ", " ;
		cout << '('; for (auto &i: path[i]){
			cout << '(' << i[0] << ", " << i[1] << ')' << ", ";
		}
		cout << ")";
		cout << '\n';
	}
}

int main(){
	ios::sync_with_stdio(false);
	cin.tie(nullptr);

	take_input();

	assert(buses == 1);
	rep(i, 0, buses){
		cur = bus[i];
		generategraph();

		auto path = solve();
		print_path(path);
	}

}
