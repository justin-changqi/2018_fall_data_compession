#include <iostream>
#include <fstream>
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <iomanip>
#include <sstream>
#include <string>

class Node
{
  public:
  std::string letter;
  int cnt;
  Node *parent;
  Node *child_r;
  Node *child_l;
  Node(std::string letter, int cnt);
  bool operator<(const  Node & other) //(1)
  {
      return cnt < other.cnt;
  }
};

class HuffmanCode {
  public:
  std::vector<Node> nodes;
  Node *root;
  HuffmanCode(std::string file);
  void printTable(std::vector<char> alphabas, std::vector<int> counts);
  std::string getSymbol(char c);
  void initNodes(std::vector<char> alphabas, std::vector<int> counts);
  void buildTree();
};
