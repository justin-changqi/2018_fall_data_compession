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
  Node *child_r;
  Node *child_l;
  Node(std::string letter, int cnt);
  Node(int cnt, Node *child_l, Node *child_r);
  bool operator<(const  Node & other) 
  {
      return cnt < other.cnt;
  }
  bool operator>=(const  Node & other) 
  {
      return cnt >= other.cnt;
  }
  bool operator==(const  Node & other) 
  {
      return cnt == other.cnt;
  }
  int operator+(const  Node & other)
  {
      return cnt + other.cnt;
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
  void mergeNodesToList(std::vector<Node> &list, int start_indx, int end_indx);
  void printTree(Node *root, int spaces);
};
