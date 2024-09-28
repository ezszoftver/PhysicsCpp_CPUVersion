#include <glm/glm.hpp>
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>

struct CollisionPoints 
{
    glm::vec3 collisionPoint;
    glm::vec3 Normal;
    float PenetrationDepth;
    bool HasCollision;
};

struct Simplex
{
private:
    std::vector<glm::vec3> m_points;
    int m_size;

public:
    Simplex()
        : m_size(0)
    {
        // Reserve space for 4 points (if necessary for performance)
        m_points.reserve(4);
    }

    Simplex& operator=(std::initializer_list<glm::vec3> list)
    {
        m_size = 0;
        m_points.clear();

        for (glm::vec3 point : list) {
            if (m_size < 4) { // Limiting to 4 points
                m_points.push_back(point);
                m_size++;
            }
        }

        return *this;
    }

    void push_front(glm::vec3 point)
    {
        if (m_size < 4) {
            m_points.insert(m_points.begin(), point);
            m_size++;
        }
        else {
            m_points = { point, m_points[0], m_points[1], m_points[2] };
        }
    }

    glm::vec3& operator[](int i) { return m_points[i]; }
    size_t size() const { return m_size; }

    auto begin() const { return m_points.begin(); }
    auto end() const { return m_points.begin() + m_size; }  // Adjusted to handle dynamic size
};

glm::vec3 FindFurthestPoint(const std::vector<glm::vec3>& m_vertices, glm::vec3 direction)
{
    glm::vec3 maxPoint;
    float maxDistance = -FLT_MAX;

    for (glm::vec3 vertex : m_vertices) 
    {
        float distance = glm::dot(vertex, direction);
        if (distance > maxDistance) 
        {
            maxDistance = distance;
            maxPoint = vertex;
        }
    }

    return maxPoint;
}

// Minkowski-differencia támogatási függvénye
glm::vec3 Support(const std::vector<glm::vec3>& colliderA, const std::vector<glm::vec3>& colliderB, const glm::vec3& direction, glm::vec3* pointA, glm::vec3* pointB)
{
    glm::vec3 furthestA = FindFurthestPoint(colliderA, direction);
    glm::vec3 furthestB = FindFurthestPoint(colliderB, -direction);

    if (pointA) *pointA = furthestA;
    if (pointB) *pointB = furthestB;

    return furthestA - furthestB; // Minkowski-differencia pontja
}

bool SameDirection(const glm::vec3& direction, const glm::vec3& ao)
{
    return glm::dot(direction, ao) > 0;
}

bool Line(Simplex& points, glm::vec3& direction)
{
    glm::vec3 a = points[0];
    glm::vec3 b = points[1];

    glm::vec3 ab = b - a;
    glm::vec3 ao = -a;

    if (SameDirection(ab, ao)) {
        direction = glm::cross(glm::cross(ab, ao), ab);
    }

    else {
        points = { a };
        direction = ao;
    }

    return false;
}

bool Triangle(Simplex& points, glm::vec3& direction)
{
    glm::vec3 a = points[0];
    glm::vec3 b = points[1];
    glm::vec3 c = points[2];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ao = -a;

    glm::vec3 abc = glm::cross(ab, ac);

    if (SameDirection(glm::cross(abc, ac), ao)) {
        if (SameDirection(ac, ao)) {
            points = { a, c };
            direction = glm::cross(glm::cross(ac, ao), ac);
        }

        else {
            return Line(points = { a, b }, direction);
        }
    }

    else {
        if (SameDirection(glm::cross(ab, abc), ao)) {
            return Line(points = { a, b }, direction);
        }

        else {
            if (SameDirection(abc, ao)) {
                direction = abc;
            }

            else {
                points = { a, c, b };
                direction = -abc;
            }
        }
    }

    return false;
}

bool Tetrahedron(Simplex& points, glm::vec3& direction)
{
    glm::vec3 a = points[0];
    glm::vec3 b = points[1];
    glm::vec3 c = points[2];
    glm::vec3 d = points[3];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ad = d - a;
    glm::vec3 ao = -a;

    glm::vec3 abc = glm::cross(ab, ac);
    glm::vec3 acd = glm::cross(ac, ad);
    glm::vec3 adb = glm::cross(ad, ab);

    if (SameDirection(abc, ao)) {
        return Triangle(points = { a, b, c }, direction);
    }

    if (SameDirection(acd, ao)) {
        return Triangle(points = { a, c, d }, direction);
    }

    if (SameDirection(adb, ao)) {
        return Triangle(points = { a, d, b }, direction);
    }

    return true;
}

bool NextSimplex(Simplex& points, glm::vec3& direction)
{
    switch (points.size()) {
    case 2: return Line(points, direction);
    case 3: return Triangle(points, direction);
    case 4: return Tetrahedron(points, direction);
    }

    // never should be here
    return false;
}

bool GJK(const std::vector<glm::vec3>& colliderA, const std::vector<glm::vec3>& colliderB, Simplex& points)
{
    glm::vec3 pointA, pointB; // A két test legközelebbi pontjai
    glm::vec3 support = Support(colliderA, colliderB, glm::vec3(1, 1, 0), &pointA, &pointB);

    // Simplex is an array of points, max count is 4
    points.push_front(support);

    // New direction is towards the origin
    glm::vec3 direction = -support;

    int iterations = 0;
    while (iterations < 64) 
    {
        iterations++;

        support = Support(colliderA, colliderB, direction, &pointA, &pointB);

        if (dot(support, direction) <= 0) {
            return false; // no collision
        }

        points.push_front(support);

        if (NextSimplex(points, direction)) {
            return true;
        }
    }

    return false;
}

void GetFaceNormals(
    const std::vector<glm::vec3>& polytope,
    const std::vector<size_t>& faces,

    std::vector<glm::vec4> *normals,
    size_t *minTriangle)
{
    float  minDistance = FLT_MAX;

    for (size_t i = 0; i < faces.size(); i += 3) {
        glm::vec3 a = polytope[faces[i]];
        glm::vec3 b = polytope[faces[i + 1]];
        glm::vec3 c = polytope[faces[i + 2]];

        glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
        float distance = dot(normal, a);

        if (distance < 0) {
            normal *= -1;
            distance *= -1;
        }

        (*normals).emplace_back(normal, distance);

        if (distance < minDistance) {
            (*minTriangle) = i / 3;
            minDistance = distance;
        }
    }
}

void AddIfUniqueEdge(
    std::vector<std::pair<size_t, size_t>>& edges,
    const std::vector<size_t>& faces,
    size_t a,
    size_t b)
{
    auto reverse = std::find(                       //      0--<--3
        edges.begin(),                              //     / \ B /   A: 2-0
        edges.end(),                                //    / A \ /    B: 0-2
        std::make_pair(faces[b], faces[a]) //   1-->--2
    );

    if (reverse != edges.end()) {
        edges.erase(reverse);
    }

    else {
        edges.emplace_back(faces[a], faces[b]);
    }
}

CollisionPoints EPA(
    const std::vector<glm::vec3>& colliderA,
    const std::vector<glm::vec3>& colliderB,
    const Simplex& simplex)
{
    std::vector<glm::vec3> polytope(simplex.begin(), simplex.end());
    std::vector<size_t> faces = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2
    };

    std::vector<glm::vec4> normals;
    size_t minFace;
    GetFaceNormals(polytope, faces, &normals, &minFace);

    glm::vec3 minNormal;
    float minDistance = FLT_MAX;

    int iterations = 0;
    glm::vec3 pointA, pointB; // A két test legközelebbi pontjai

    while (minDistance == FLT_MAX && iterations < 64)
    {
        minNormal = glm::vec3(normals[minFace]);
        minDistance = normals[minFace].w;

        glm::vec3 support = Support(colliderA, colliderB, minNormal, &pointA, &pointB);
        float sDistance = glm::dot(minNormal, support);

        if (abs(sDistance - minDistance) > 0.001f) {
            minDistance = FLT_MAX;

            std::vector<std::pair<size_t, size_t>> uniqueEdges;

            for (size_t i = 0; i < normals.size(); i++) {
                if (SameDirection(normals[i], support)) {
                    size_t f = i * 3;

                    AddIfUniqueEdge(uniqueEdges, faces, f, f + 1);
                    AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
                    AddIfUniqueEdge(uniqueEdges, faces, f + 2, f);

                    faces[f + 2] = faces.back(); faces.pop_back();
                    faces[f + 1] = faces.back(); faces.pop_back();
                    faces[f] = faces.back(); faces.pop_back();

                    normals[i] = normals.back(); // pop-erase
                    normals.pop_back();

                    i--;
                }
            }

            std::vector<size_t> newFaces;
            for (const auto& edge : uniqueEdges) {
                size_t edgeIndex1 = edge.first;
                size_t edgeIndex2 = edge.second;

                newFaces.push_back(edgeIndex1);
                newFaces.push_back(edgeIndex2);
                newFaces.push_back(polytope.size());
            }

            polytope.push_back(support);

            std::vector<glm::vec4> newNormals;
            size_t newMinFace;
            GetFaceNormals(polytope, newFaces, &newNormals, &newMinFace);

            float oldMinDistance = FLT_MAX;
            for (size_t i = 0; i < normals.size(); i++) {
                if (normals[i].w < oldMinDistance) {
                    oldMinDistance = normals[i].w;
                    minFace = i;
                }
            }

            if (newNormals[newMinFace].w < oldMinDistance) {
                minFace = newMinFace + normals.size();
            }

            faces.insert(faces.end(), newFaces.begin(), newFaces.end());
            normals.insert(normals.end(), newNormals.begin(), newNormals.end());
        }
    }

    CollisionPoints points;

    if (minDistance == FLT_MAX)
    {
        points.HasCollision = false;
    }
    else
    {
        points.Normal = minNormal;
        points.PenetrationDepth = minDistance;
        points.HasCollision = true;

        // Világkoordinátás ütközési pont kiszámítása
        points.collisionPoint = (pointA + pointB) / 2.0f;
    }

    return points;
}

int main() {
    glm::vec3 position1 = glm::vec3(100, 100, 100);
    std::vector<glm::vec3> shape1 = {
        position1 + glm::vec3(-1, -1, -1), position1 + glm::vec3(1, -1, -1), position1 + glm::vec3(1, 1, -1), position1 + glm::vec3(-1, 1, -1), // Alsó négyzet
        position1 + glm::vec3(-1, -1, 1), position1 + glm::vec3(1, -1, 1), position1 + glm::vec3(1, 1, 1), position1 + glm::vec3(-1, 1, 1)      // Felső négyzet
    };

    // Shape2: Egy másik 3D-s kocka, kicsit eltolva az origóhoz képest, hogy érintkezzen az elsővel
    glm::vec3 position2 = glm::vec3(100.0f, 101, 100);
    std::vector<glm::vec3> shape2 = {
        position2 + glm::vec3(-1, -1, -1), position2 + glm::vec3(1, -1, -1), position2 + glm::vec3(1, 1, -1), position2 + glm::vec3(-1, 1, -1), // Alsó négyzet
        position2 + glm::vec3(-1, -1, 1), position2 + glm::vec3(1, -1, 1), position2 + glm::vec3(1, 1, 1), position2 + glm::vec3(-1, 1, 1)      // Felső négyzet
    };

    Simplex simplex;
    if (true == GJK(shape1, shape2, simplex)) 
    {
        CollisionPoints points = EPA(shape1, shape2, simplex);

        printf("Collision detected\n");
        printf(" -> Collision Point: %.2f %.2f %.2f\n", points.collisionPoint.x, points.collisionPoint.y, points.collisionPoint.z);
        printf(" -> Collision Normal: %.2f %.2f %.2f\n", points.Normal.x, points.Normal.y, points.Normal.z);
        printf(" -> Penetration Depth: %.2f", points.PenetrationDepth);
    }
    else 
    {
        printf("No collision");
    }

    return 0;
}