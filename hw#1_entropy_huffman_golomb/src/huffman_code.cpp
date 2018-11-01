#include "huffman_code.hpp"

Node::Node(std::string letter, int cnt)
{
  this->letter = letter;
  this->cnt = cnt;
  this->parent = NULL;
  this->child_l = NULL;
  this->child_r = NULL;
}

Node::Node(int cnt, Node *child_l, Node *child_r)
{
  this->cnt = cnt;
  this->parent = NULL;
  this->child_l = child_l;
  this->child_r = child_r;
}

HuffmanCode::HuffmanCode( std::string file ) 
{
  std::vector<char> alphabas;
  std::vector<int> counts;
  std::ifstream infile;
  char letter[0];
  infile.open (file, std::ios::app);
  while ( infile.peek()  != EOF ) {
    infile.read (letter, 1);
    // counting letter
    std::vector<char>::iterator it;
    it = std::find (alphabas.begin(), alphabas.end(), *letter);
    if (it != alphabas.end())
    {
      size_t index = it - alphabas.begin();
      counts[index] += 1;
    }
    else
    {
      if(letter[0] != 0x0a)
      {
        alphabas.push_back(letter[0]);
        counts.push_back(1);
      }
    }
  }
  infile.close();
  this->printTable(alphabas, counts);
  this->initNodes(alphabas, counts);
  this->buildTree();
}

void HuffmanCode::printTable(std::vector<char> alphabas, std::vector<int> counts)
{
  std::cout << std::setw(15) << "Alphaba";
  for (int i = 0; i < alphabas.size() / 2; i++)
  {
    std::cout << std::setw(5) << this->getSymbol(alphabas[i]);
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = 0; i < alphabas.size() / 2; i++)
  {
    std::cout << std::setw(5) << counts[i];
  }
  std::cout << std::endl << std::endl;;
  std::cout << std::setw(15) << "Alphaba";
  for (int i = alphabas.size() / 2; i < alphabas.size(); i++)
  {
    std::cout << std::setw(5) << this->getSymbol(alphabas[i]);
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = alphabas.size() / 2; i < alphabas.size(); i++)
  {
    std::cout << std::setw(5) << counts[i];
  }
  std::cout << std::endl;
}

std::string HuffmanCode::getSymbol(char c)
{
  switch(c)
  {
    case 0x0d:
      return "CR";
      break;
    case 0x20: 
      return "SP";
      break;
    default:
      std::string s;
      s += c;
      return s;
      break;
  }
}

void HuffmanCode::initNodes(std::vector<char> alphabas, std::vector<int> counts)
{
  for (int i = 0; i < alphabas.size(); i++)
  {
    Node n(this->getSymbol(alphabas[i]), counts[i]);
    this->nodes.push_back(n);
  }
}

void HuffmanCode::buildTree()
{
  std::sort(this->nodes.begin(), this->nodes.end());
  for (std::vector<Node>::iterator it=this->nodes.begin(); it!=this->nodes.end(); ++it)
      std::cout << ' ' << it->letter << ":" << it->cnt;
    std::cout << '\n';
  std::cout << "Start Tree" << std::endl;
  do
  {
    int node_merge_cnt = 1;
    Node node =  this->nodes[0];
    for (int i = 1; i < this->nodes.size(); i++)
    {
      Node node_next =  this->nodes[i];
      if (node == node_next)
      {
        node_merge_cnt++;
      }
      else
      {
        break;
      }
    }
    if (node_merge_cnt == 1)
    {
      node_merge_cnt = 2;
    }
    else
    {
      node_merge_cnt = (node_merge_cnt / 2) * 2;
    }
    std::vector<Node> merge_nodes;
    for (int i = 0; i < node_merge_cnt; i++)
    {
      merge_nodes.push_back(this->nodes[0]);
      this->nodes.erase (this->nodes.begin());
      // this->nodes.erase (this->nodes.begin()+i+1);
    } 
    // Merge Node then put into node list
    this->mergeNodesToList(this->nodes, merge_nodes);
    // for (std::vector<Node>::iterator it=this->nodes.begin(); it!=this->nodes.end(); ++it)
    //   std::cout << ' ' << it->letter << ":" << it->cnt;
    // std::cout << '\n';
    //  for (int i = 0; i < merge_nodes.size(); i++)
    // {
    //     std::cout << ' ' << merge_nodes[i].letter << ":" << merge_nodes[i].cnt;
    // }
    // std::cout << std::endl;
    for (int i = 0; i < this->nodes.size(); i++)
    {
        std::cout << ' ' << this->nodes[i].letter << ":" << this->nodes[i].cnt;
    }
     std::cout << std::endl;
  }while(this->nodes.size() != 1);
  this->root = &this->nodes[0];
}

void HuffmanCode::mergeNodesToList(std::vector<Node> &list, std::vector<Node> &nodes)
{
  for (int i = 0; i < nodes.size(); i = i + 2)
  {
    Node n(nodes[i]+nodes[i+1], &nodes[i], &nodes[i+1]); 
    if (list.size() > 1)
    {
      bool inserted = false;
      for (int j = 0; j < list.size(); j++)
      {
        if (list[j] >= n)
        {
          list.insert(list.begin()+j, n);
          inserted = true;
          break;
        }
      }
      if (!inserted)
      {
        list.push_back(n);
      }
    }
    else
    {
      // std::cout << "final n" << std::endl;
      list.push_back(n);
    }
  }
}

void HuffmanCode::printTree()
{
  std::cout << root->cnt << std::endl;
}

int main(int argc, char const *argv[])
{
  // HuffmanCode huffman_code("../santaclaus.txt");
  HuffmanCode huffman_code("../test.txt");
  huffman_code.printTree();
  return 0;
}
