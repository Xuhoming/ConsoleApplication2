#include<iostream>
using namespace std;
char t[10];
double x, y, z;
double maxx, maxy, maxz;
double minx, miny, minz;
double maxn;
void suan() {
	maxx = -10000010;
	maxy = -10000010;
	maxz = -10000010;
	minx = 1000010;
	miny = 1000010;
	minz = 1000010;
	//cout << t[1];
	for (int i = 1;;i++) {
		char t[10];
		memset(t, 0, sizeof(t));

		scanf("%s", t);//cout << t << endl;
		if (t[1] == 'n') {
			cin >> x >> y >> z;
			//cout << "1" << endl;
			continue;
		};
		if (t[0] != 'v') break;
		if (t[0] == 'v') {
			double r;
			cin >> x >> y >> z ;
			if (x > maxx) maxx = x;
			//if (y > maxy) maxy = y;
			//if (z > maxz) maxz = z;
			if (x < minx) minx = x;
			//if (y < miny) miny = y;
			//if (z < minz) minz = z;

		}

	}
	maxn = maxx - minx;
	cout << maxn;
	return;

}
void cheng() {
	double g;
	cin >> g;
for (int i = 1;;i++) {
		char t[10];
		memset(t, 0, sizeof(t));

		scanf("%s", t);//cout << t << endl;
		if (t[1] == 'n') {
			cin >> x >> y >> z;
			cout << t <<" "<< x <<" "<< y <<" "<< z<<endl;
			//cout << "1" << endl;
			continue;
		};
		if (t[0] != 'v') break;
		if (t[0] == 'v') {
			double r;
			cin >> x >> y >> z;// >> r >> r >> r;
			
			x = x / g;
			y = y /g;
			z = z / g;
			r = r /g;
			cout << t << " " << x << " " << y << " " << z << endl;//<< " " << r << " " << r << " " << r << endl;
		}

	}
return;

}
int main() {
	freopen("leigu.txt", "r", stdin);
	freopen("leigu.out", "w", stdout);
    //suan();
	cheng();
	
	//cout << maxn << endl;
	return 0;
}