#ifndef PATHHELPER_H
#define PATHHELPER_H

#include <vector>

class PathHelper
{
public:
    // Traversal Path
    static std::vector<int> GetPath(int end, const std::vector<int>& cameFrom);
    static std::vector<int> GetPath(int start,
                                    int end,
                                    const std::vector<std::vector<int>>& cameFrom);

    // Topological sort helper methods
    static int GetUnvisitedNode(const std::vector<int>& visited);
    static int GetUnanalyzedNeighbour(const std::vector<int>& neighbours,
                                      const std::vector<int>& visited);

    // Connected components helper method
    static int GetUnvisitedNode(const std::vector<bool>& visited);
};

#endif // PATHHELPER_H
