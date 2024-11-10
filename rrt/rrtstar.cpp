#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

using namespace cv;
using namespace std;

Node *getRandomNode();                                                  // node oluştur
double distance(Node *n1, Node *n2);                                    // mesafe bul
Node *getNearestNode(vector<Node *> &nodes, Node *randomNode);          // en yakın node
Node *getNewNode(Node *nearestNode, Node *randomNode, double stepSize); // yeni düğüm
bool isCollision(Node *n1, Node *n2, vector<Rect> &obstacles);          // engele çarptı mı?
void rewire(Node *newNode, vector<Node *> &nodes, double stepSize);     // yeniden bağla
void drawPath(Mat &img, vector<Node *> &path);                          // yolu çiz

// Node Yapısı
struct Node
{
    int x, y;
    double cost;
    Node *parent;
    Node(int x, int y) : x(x), y(y), cost(0), parent(nullptr) {}
};

// Rastgele Sayı
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis(0, 499);

Node *getRandomNode()
{
    return new Node(dis(gen), dis(gen));
}

double distance(Node *n1, Node *n2)
{
    return sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
}

// En Yakın Düğümü Bul
Node *getNearestNode(vector<Node *> &nodes, Node *randomNode)
{
    Node *nearestNode = nodes[0];
    double minDist = distance(nearestNode, randomNode);
    for (auto &node : nodes)
    {
        double dist = distance(node, randomNode);
        if (dist < minDist)
        {
            nearestNode = node;
            minDist = dist;
        }
    }
    return nearestNode;
}

Node *getNewNode(Node *nearestNode, Node *randomNode, double stepSize)
{
    double theta = atan2(randomNode->y - nearestNode->y, randomNode->x - nearestNode->x);
    int newX = nearestNode->x + stepSize * cos(theta);
    int newY = nearestNode->y + stepSize * sin(theta);
    Node *newNode = new Node(newX, newY);
    newNode->cost = nearestNode->cost + stepSize;
    newNode->parent = nearestNode;
    return newNode;
}

bool isCollision(Node *n1, Node *n2, vector<Rect> &obstacles)
{
    for (auto &obstacle : obstacles)
    {
        // Engelle çarpışmayı kontrol et
        if (obstacle.contains(Point(n2->x, n2->y)) || obstacle.contains(Point(n1->x, n1->y)))
        {
            return true; // Çarpışma var
        }
    }
    return false; // Çarpışma yok
}

void rewire(Node *newNode, vector<Node *> &nodes, double stepSize)
{
    for (auto &node : nodes)
    {
        if (distance(node, newNode) <= stepSize && newNode->cost + stepSize < node->cost)
        {
            node->parent = newNode;
            node->cost = newNode->cost + stepSize;
        }
    }
}

vector<Node *> planPath(Node *start, Node *goal, vector<Rect> &obstacles, double stepSize, int maxIter)
{
    vector<Node *> nodes;
    nodes.push_back(start);

    for (int i = 0; i < maxIter; ++i)
    {
        Node *randomNode = getRandomNode();
        Node *nearestNode = getNearestNode(nodes, randomNode);
        Node *newNode = getNewNode(nearestNode, randomNode, stepSize);

        if (!isCollision(nearestNode, newNode, obstacles)) // Çarpışma kontrolü
        {
            nodes.push_back(newNode);
            rewire(newNode, nodes, stepSize);

            if (distance(newNode, goal) <= stepSize)
            {
                goal->parent = newNode;
                nodes.push_back(goal);
                break;
            }
        }
        delete randomNode;
    }

    vector<Node *> path;
    Node *node = goal;
    while (node->parent != nullptr)
    {
        path.push_back(node);
        node = node->parent;
    }
    path.push_back(start);
    return path;
}

void drawPath(Mat &img, vector<Node *> &path)
{
    for (int i = 0; i < path.size() - 1; ++i)
    {
        line(img, Point(path[i]->x, path[i]->y), Point(path[i + 1]->x, path[i + 1]->y), Scalar(255, 0, 0), 2);
    }
}

int main()
{
    Mat img = Mat::ones(500, 500, CV_8UC3) * 255;

    Node *start = new Node(50, 50);
    Node *goal = new Node(450, 450);

    vector<Rect> obstacles = {Rect(150, 150, 100, 50), // Dikdörtgen 1
                              Rect(300, 200, 120, 60), // Dikdörtgen 2
                              Rect(100, 350, 130, 60), // Dikdörtgen 3
                              Rect(250, 100, 90, 40)}; // Dikdörtgen 4

    // Dikdörtgenleri çiz
    for (auto &obstacle : obstacles)
    {
        rectangle(img, obstacle, Scalar(0, 0, 255), -1); // Kırmızı renk ile doldur
    }

    double stepSize = 10.0;
    int maxIter = 1000;

    vector<Node *> path = planPath(start, goal, obstacles, stepSize, maxIter);

    // ağacı çiz
    for (auto &node : path)
    {
        if (node->parent)
        {
            line(img, Point(node->x, node->y), Point(node->parent->x, node->parent->y), Scalar(0, 255, 0), 1);
        }
    }

    drawPath(img, path);

    imshow("RRT*", img);
    waitKey(0);

    // serbest bırakma
    for (auto &node : path)
    {
        delete node;
    }
    delete start;
    delete goal;

    return 0;
}
