

#include <stdio.h>
#include <math.h>


// for defining a key for NavMesh::edges map 
long long int CantorPair(long long int x, long long int y) {
	long long int res = (x + y) * (x + y + 1) / 2 + y;
	return res; 
}



// each point has an id, making it significantly easier to distinguish them in the course of BowyerWatson(), 
// e.g., it allows to identify which triangles share vertices with the super triangle at the end of the algorithm, or whether two triangles share an edge 
struct Point {
	float x;
	float y;
	int id;
};

// used to check whether an intersection point of two lines lies on either of them 
int Between(struct Point pt, struct Point s, struct Point e) {

	float max_x;
	float min_x;
	float max_y;
	float min_y;

	if (s.x >= e.x) {
		max_x = s.x;
		min_x = e.x;
	}
	else {
		max_x = e.x;
		min_x = s.x;
	}
	if (s.y >= e.y) {
		max_y = s.y;
		min_y = e.y;
	}
	else {
		max_y = e.y;
		min_y = s.y;
	}
	if ((int)pt.x <= (int)max_x && (int)pt.x >= (int)min_x && (int)pt.y <= (int)max_y && (int)pt.y >= (int)min_y) return 1;
	else return 0; 

}


// for checking edge-edge intersections in edge-obstacle intersection check 
struct ObsEdge {
	struct Point start;
	struct Point end; 
	float slope;
	float y_intercept;
};

// Helper struct for managing polygons and triangle edges in determining unique edges and creating new triangles out of them and the next mesh point 
struct PolyEdge {
	int start;
	int end;
	int unique;
};

// Wrapper struct to return Bowyer-Watson's results to the outer scope 
struct Edge {
	int start;
	int end;
	float weight; // the distance between start and end 
	int last;
};

// get distance between two points 
float GetWeight(struct Point one, struct Point two) {

	struct Point diff_vect;
	diff_vect.x = one.x - two.x;
	diff_vect.y = one.y - two.y;

	return (float)sqrt(diff_vect.x * diff_vect.x + diff_vect.y * diff_vect.y);
}

int EqualsEdge(struct Edge one, struct Edge two) {
	if ((one.start == two.end || one.start == two.start) && (one.end == two.start || one.end == two.end)) return 1;
	else return 0;
}

int EqualsPair(struct PolyEdge one, struct PolyEdge two) {
	if ((one.start == two.end || one.start == two.start) && (one.end == two.start || one.end == two.end)) return 1;
	else return 0;
}



// struct for obstacles passed into BowyerWatson() 
// invalid edges intersect with a rect obstacle and are removed from the return array 
struct Rect {
	struct ObsEdge edges[4];
};

// helper to IntersectRect()
int IntersectsEdge(struct ObsEdge edge, struct ObsEdge other) {

	if (fabs(edge.slope - other.slope) < 1e-14) return 0;

	struct Point intersect_pt;
	if (isinf(other.slope) || other.slope == 0.0f) {
		if (isinf(other.slope)) {
			intersect_pt.x = other.start.x;
			intersect_pt.y = edge.slope * intersect_pt.x + edge.y_intercept;
		}
		else {
			intersect_pt.x = (other.y_intercept - edge.y_intercept) / (edge.slope - other.slope);
			intersect_pt.y = other.start.y;
		}
	}

	else if (isinf(edge.slope) || edge.slope == 0.0f) {
		if (isinf(edge.slope)) {
			intersect_pt.x = edge.start.x;
			intersect_pt.y = edge.slope * intersect_pt.x + edge.y_intercept;
		}
		else {
			intersect_pt.x = (edge.y_intercept - other.y_intercept) / (other.slope - edge.slope);
			intersect_pt.y = edge.start.y;
		}
	}
	else {
		intersect_pt.x = (other.y_intercept - edge.y_intercept) / (edge.slope - other.slope);
		intersect_pt.y = edge.slope * intersect_pt.x + edge.y_intercept;
	}

	if (Between(intersect_pt, edge.start, edge.end) && Between(intersect_pt, other.start, other.end)) return 1;
	else return 0; 
}

// helper to ObstacleCheck()
int IntersectsRect(struct ObsEdge edge, struct Rect obs) {

	edge.slope = (edge.end.y - edge.start.y) / (edge.end.x - edge.start.x);
	edge.y_intercept = edge.start.y - edge.slope * edge.start.x;

	for (int i = 0; i < 4; ++i) if (IntersectsEdge(edge, obs.edges[i])) return 1;

	return 0; 
}

int ObstacleCheck(struct ObsEdge edge, struct Rect* obstacles, int obs_size) {
	for (int i = 0; i < obs_size; ++i) {
		if (IntersectsRect(edge, obstacles[i])) return 0;
	}
	return 1; 
}

// generate slopes and y intercepts for all obstacle edges; called at the beginning of BowyerWatson() 
void GenObstacleEdges(struct Rect* obstacles, int obs_size) {
	for (int i = 0; i < obs_size; ++i) {
		for (int j = 0; j < 4; ++j) {
			obstacles[i].edges[j].slope = (obstacles[i].edges[j].end.y - obstacles[i].edges[j].start.y) / (obstacles[i].edges[j].end.x - obstacles[i].edges[j].start.x);
			obstacles[i].edges[j].y_intercept = obstacles[i].edges[j].start.y - obstacles[i].edges[j].slope * obstacles[i].edges[j].start.x;
		}
	}
}



// Circumcircle struct and methods 
struct Circumcircle {
	float radius;
	float centre_x;
	float centre_y;
};

int CircumcircleContains(struct Point pt, struct Circumcircle circumcircle) {

	struct Point to_centre;
	to_centre.x = pt.x - circumcircle.centre_x;
	to_centre.y = pt.y - circumcircle.centre_y;

	float len = (float)sqrt(to_centre.x * to_centre.x + to_centre.y * to_centre.y);

	if (len < circumcircle.radius) return 1;
	else return 0;
}



// Triangle struct and methods 
// vertices defined counterclockwise 
struct Triangle {
	struct Point vertices[3];
	struct PolyEdge edges[3];
	struct Circumcircle circumcircle;
};

struct Circumcircle GetCircumcircle(struct Triangle triangle) {

	struct Circumcircle res;

	struct Point A = triangle.vertices[0];
	struct Point B = triangle.vertices[1];
	struct Point C = triangle.vertices[2];

	float det = (A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y)) * 2;

	res.centre_x = (1 / det) * ((A.x * A.x + A.y * A.y) * (B.y - C.y) + (B.x * B.x + B.y * B.y) * (C.y - A.y) + (C.x * C.x + C.y * C.y) * (A.y - B.y));
	res.centre_y = (1 / det) * ((A.x * A.x + A.y * A.y) * (C.x - B.x) + (B.x * B.x + B.y * B.y) * (A.x - C.x) + (C.x * C.x + C.y * C.y) * (B.x - A.x));
	struct Point diff;
	diff.x = A.x - res.centre_x;
	diff.y = A.y - res.centre_y;
	res.radius = sqrt(diff.x * diff.x + diff.y * diff.y);

	return res;
}

struct Triangle GetSuperTriangle(float excircle_rad, float excircle_pos_x, float excircle_pos_y, int pt_c) {

	struct Triangle triangle;
	float side_len = 2 * excircle_rad * (float)sqrt(3);

	struct Point top_pt = { excircle_pos_x, excircle_pos_y - (excircle_rad + side_len / (float)sqrt(3)) };
	top_pt.id = pt_c;
	triangle.vertices[0] = top_pt;

	struct Point left_pt = { excircle_pos_x - side_len / 2.0f, excircle_pos_y + excircle_rad };
	left_pt.id = pt_c + 1;
	triangle.vertices[1] = left_pt;

	struct Point right_pt = { excircle_pos_x + side_len / 2.0f, excircle_pos_y + excircle_rad };
	right_pt.id = pt_c + 2;
	triangle.vertices[2] = right_pt;

	struct PolyEdge one = { triangle.vertices[0].id, triangle.vertices[1].id, 0 };
	triangle.edges[0] = one;
	struct PolyEdge two = { triangle.vertices[1].id, triangle.vertices[2].id, 0 };
	triangle.edges[1] = two;
	struct PolyEdge three = { triangle.vertices[2].id, triangle.vertices[0].id, 0 };
	triangle.edges[2] = three;

	triangle.circumcircle = GetCircumcircle(triangle);

	return triangle;
}

int HasEdge(struct PolyEdge edge, struct Triangle tr) {
	if (EqualsPair(edge, tr.edges[0]) || EqualsPair(edge, tr.edges[1]) || EqualsPair(edge, tr.edges[2])) return 1;
	else return 0;
}

// triangles array methods 
void RemoveTriangle(int arr_size, struct Triangle* t_arr, int at_index) {
	for (int i = at_index; i < arr_size - 1; ++i) t_arr[i] = t_arr[i + 1];
}

struct Triangle* ResizeTrianglesArray(int arr_size, struct Triangle* t_arr) {
	int n_size = arr_size << 1;
	struct Triangle* n_arr = (struct Triangle*)malloc(sizeof(struct Triangle) * n_size);

	if (n_arr == NULL) {
		printf("malloc for triangle array resize failed");
		return NULL;
	}

	for (int i = 0; i < arr_size; ++i) n_arr[i] = t_arr[i];

	free(t_arr);

	return n_arr;
}


// Triangulation algorithm 
struct Edge* BowyerWatson(int pt_count, struct Point* points, float excircle_rad, float excircle_pos_x, float excircle_pos_y, struct Rect* obstacles, int obs_count) {

	if (pt_count <= 2 || excircle_rad <= 0) return NULL; 

	int tr_arr_size = pt_count * 3;
	float resize_threshold = 0.9f; // to check if the triangles array needs resizing 
	struct Triangle* triangles = (struct Triangle*)malloc(sizeof(struct Triangle) * tr_arr_size);

	if (triangles == NULL) {
		printf("triangles malloc failed");
		return NULL;
	}

	GenObstacleEdges(obstacles, obs_count); // generate obstacle edges' slopes and y_intercepts 

	// I pass pt_count to super-triangle so that its Points represent the last three indices in the points array - indices at pt_count, pt_count + 1 and pt_count + 2
	struct Triangle super_triangle = GetSuperTriangle(excircle_rad, excircle_pos_x, excircle_pos_y, pt_count);
	triangles[0] = super_triangle;
	for (int i = 0; i < 3; ++i) points[pt_count + i] = super_triangle.vertices[i];
	int tr_count = 1; 
	
	// populates poly_edges lookup array ahead of edge-uniqueness check 
	struct PolyEdge null_edge = { -1, -1, 1 };

	// Main increment loop; iterates over all points of the mesh 
	for (int pt_i = 0; pt_i < pt_count; ++pt_i) {

		struct Triangle* bad_tr = (struct Triangle*)malloc(sizeof(struct Triangle) * tr_count);

		if (bad_tr == NULL) {
			printf("bad triangles malloc failed");
			return NULL;
		}

		int bad_tr_count = 0;
		// get all the "bad triangles" (bad because their circumcircle contains points[pt_i]) to define the polygon hole in which points[pt_i] will be triangulated
		for (int j = 0; j < tr_count; ++j) {

			if (CircumcircleContains(points[pt_i], triangles[j].circumcircle)) {
				bad_tr[bad_tr_count++] = triangles[j];
				RemoveTriangle(tr_count, triangles, j);
				--j;
				--tr_count;
			}

		}

		// representing the polygon hole with poly_edges lookup array, consisting of all unique edges of the bad triangles
		int edges_arr_size = bad_tr_count * 3; 
		struct PolyEdge* poly_edges = (struct PolyEdge*)malloc(sizeof(struct PolyEdge) * edges_arr_size);
		for (int i = 0; i < edges_arr_size; ++i) poly_edges[i] = null_edge; 

		if (poly_edges == NULL) {
			printf("poly edges malloc failed");
			return NULL;
		}

		if (bad_tr_count == 1) { // if only one bad triangle, all edges are unique 
			for (int i = 0; i < 3; ++i) poly_edges[i] = bad_tr[0].edges[i];
		}
		else {
			// checking if the edges are unique 
			for (int i = 0; i < bad_tr_count; ++i) {
				for (int j = 0; j < 3; ++j) {

					struct PolyEdge next = bad_tr[i].edges[j];
					int lookup_index = (next.start ^ next.end) * 7 % edges_arr_size;

					// open addressing with linear probing of poly_edges until either an empty slot is found, or an equal edge is found (increment unique struct member, to then 
					// skip the edges for which (unique > 0), when triangulating the point)
					while (1) {
						if (poly_edges[lookup_index].start == -1) {
							poly_edges[lookup_index] = next;
							break;
						}
						if (EqualsPair(poly_edges[lookup_index], next)) {
							++poly_edges[lookup_index].unique;
							break; 
						}
						lookup_index = (lookup_index + 1) % edges_arr_size;
					}
				}
			}
		}

		// create new triangles out of the polygon and add them to triangles 
		for (int i = 0; i < edges_arr_size; ++i) {

			if (poly_edges[i].unique > 0) continue;

			struct Triangle next;
			next.vertices[0] = points[pt_i];
			// the id of a Point also happens to be the index of this Point in the cpoints array; PolyEdge stores the ids of these points instead of the Point instances themselves 
			next.vertices[1] = points[poly_edges[i].start];
			next.vertices[2] = points[poly_edges[i].end];

			// generate the edges 
			struct PolyEdge one = { next.vertices[0].id, next.vertices[1].id, 0 };
			next.edges[0] = one;
			struct PolyEdge two = { next.vertices[1].id, next.vertices[2].id, 0 };
			next.edges[1] = two;
			struct PolyEdge three = { next.vertices[2].id, next.vertices[0].id, 0 };
			next.edges[2] = three;

			next.circumcircle = GetCircumcircle(next);
			triangles[tr_count++] = next;

			if (tr_count / (float)tr_arr_size > resize_threshold) { // in case the new triangle addition crossed the resize_threshold 
				triangles = ResizeTrianglesArray(tr_arr_size, triangles);
				tr_arr_size = tr_arr_size << 1;
			}
		}

		free(poly_edges);
		free(bad_tr);
	}

	// loop through all triangles and remove any that share vertices with the super triangle 
	for (int i = 0; i < tr_count; ++i) {

		if (triangles[i].vertices[0].id >= pt_count || triangles[i].vertices[1].id >= pt_count || triangles[i].vertices[2].id >= pt_count) {
			RemoveTriangle(tr_count, triangles, i);
			--tr_count; 
			--i; 
		}
	}
	
	// assemble the return array // most edges are duplicated, and are skipped using a lookup table similar to the way polygon hole edges are processed in the main loop 
	int edge_count = 0; 
	int lookup_size = tr_count * 3;

	if (tr_count == 0) {
		free(triangles);
		return NULL;
	}

	struct Edge* res = (struct Edge*)malloc(sizeof(struct Edge) * lookup_size + sizeof(struct Edge));
	struct PolyEdge* lookup = (struct PolyEdge*)malloc(sizeof(struct PolyEdge) * lookup_size);
	for (int i = 0; i < lookup_size; ++i) lookup[i] = null_edge;

	if (lookup == NULL || res == NULL) {
		if (lookup != NULL) free(lookup);
		if (res != NULL) free(res);
		free(triangles);
		return NULL;
	}

	printf("Removing invalid nodes...\n");

	for (int i = 0; i < tr_count; ++i) {
		for (int j = 0; j < 3; ++j) {

			struct PolyEdge next = triangles[i].edges[j];

			int hash = next.start ^ next.end;
			int lookup_index = ((hash << 4) + hash) % lookup_size; 
			int skip = 0; 
			while (1) {
				if (lookup[lookup_index].start == -1) break;
				if (EqualsPair(lookup[lookup_index], next)) {
					skip = 1; 
					break;
				}
				lookup_index = (lookup_index + 1) % lookup_size;
			}
			if (skip) continue;

			lookup[lookup_index] = next;

			struct ObsEdge edge = { points[next.start], points[next.end], 0.0f, 0.0f };
			if (!ObstacleCheck(edge, obstacles, obs_count)) continue;  

			struct Edge res_edge = { next.start, next.end, GetWeight(points[next.start], points[next.end]), 0};
			res[edge_count++] = res_edge;
		}
	}

	if (edge_count == 0) printf("No valid edges remaining\n");

	struct Edge last;
	last.start = -1; 
	last.end = -1; 
	last.last = 1; 
	res[edge_count] = last; // set a sentinel value for the outer scope to check if the return array is over 
	
	free(lookup);
	free(triangles); 
	
	return res; 
}

