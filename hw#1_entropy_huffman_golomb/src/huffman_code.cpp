#include "huffman_code.hpp"

Node::Node(std::string letter, int cnt)
{
  this->letter = letter;
  this->cnt = cnt;
  this->child_l = NULL;
  this->child_r = NULL;
}

Node::Node(int cnt, Node *child_l, Node *child_r)
{
  this->cnt = cnt;
  // this->parent = NULL;
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
  // for (std::vector<Node>::iterator it=this->nodes.begin(); it!=this->nodes.end(); ++it)
  //     std::cout << ' ' << it->letter << ":" << it->cnt;
  //   std::cout << '\n';
  // std::cout << "Start Tree" << std::endl;
  int start_index = 0;
  while(this->nodes.size()-1 > start_index)
  {
    int node_merge_cnt = 1;
    Node node =  this->nodes[start_index];
    for (int i = start_index; i < this->nodes.size(); i++)
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
    for (int i = start_index; i < start_index+node_merge_cnt; i++)
    {
      merge_nodes.push_back(this->nodes[i]);
      // this->nodes.erase (this->nodes.begin());
      // this->nodes.erase (this->nodes.begin()+i+1);
    } 
    // Merge Node then put into node list
    this->mergeNodesToList(this->nodes, start_index, start_index+node_merge_cnt);
    start_index += node_merge_cnt;
    // for (int i = 0; i < this->nodes.size(); i++)
    // {
    //     std::cout << ' ' << this->nodes[i].letter << ":" << this->nodes[i].cnt;
    // }
    //  std::cout << std::endl;
    // std::cout << start_index << std::endl;
  }
  this->root = &this->nodes.back();
  // std::cout << this->nodes.size() << std::endl;
}

void HuffmanCode::mergeNodesToList(std::vector<Node> &list,
                                   int start_indx, int end_indx) 
{
  for (int i = start_indx; i < end_indx; i = i + 2)
  {
    Node n(list[i]+list[i+1], &list[i], &list[i+1]); 
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

void HuffmanCode::printTree(Node *root, int spaces)
{
  if(root != NULL)
  {
    this->printTree(root->child_r, spaces + 5);
    for(int i = 0; i < spaces; i++)
      std::cout << ' ';
    std::cout << "   " << root->cnt << std::endl;
    this->printTree(root->child_l, spaces + 5);
  }
  else
  {
    return;
  }
}

int main(int argc, char const *argv[])
{
  // HuffmanCode huffman_code("../santaclaus.txt");
  HuffmanCode huffman_code("../test.txt");
  std::cout << std::endl << "=================== Huffman Tree ================" << std::endl;
  // HuffmanCode huffman_code("./hw#1_entropy_huffman_golomb/test.txt");
  huffman_code.printTree(huffman_code.root, 1);
  std::cout << std::endl << "===================================" << std::endl;
  return 0;
}
