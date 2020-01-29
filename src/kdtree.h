/* \author Aaron Brown */
/* modified for by S M Al Mahi for Lidar Obstacle detection project */
// Quiz on implementing kd tree
#ifndef KDTREE_H_
#define KDTREE_H_

// Structure to represent node of kd tree
template<typename PointT>
struct KdTree {
    struct Node {
        PointT point;
        int id;
        Node *left;
        Node *right;

        Node(PointT arr, int setId)
                : point(arr), id(setId), left(NULL), right(NULL) {}
    };

    Node* root;

    KdTree()
            : root(NULL) {}

    void insertHelper(Node **node, int depth, PointT point, int id) {
        // Tree is empty
        if (*node == NULL)
            *node = new Node(point, id);
        else {
            // calculate current dimension
            uint cd = depth % 3;
            if (cd == 0) // split on x axis
                if (point.x < ((*node)->point.x))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            if (cd == 1) // split on y axis
                if (point.y < ((*node)->point.y))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            if (cd == 2) // split on z axis
                if (point.z < ((*node)->point.z))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }

    void insert(PointT point, int id) {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id);

    }

    void searchHelper(PointT target, Node *node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) &&
                (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))) {
                float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) +
                                      (node->point.y - target.y) * (node->point.y - target.y) +
                                      (node->point.z - target.z) * (node->point.z - target.z));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            //check accross boundary
            int cd = depth % 3;
            if (cd == 0) {
                if ((target.x - distanceTol) < node->point.x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.x + distanceTol) > node->point.x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            if (cd == 1) {
                if ((target.y - distanceTol) < node->point.y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.y + distanceTol) > node->point.y)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            if (cd == 2) {
                if ((target.z - distanceTol) < node->point.z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.z + distanceTol) > node->point.z)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }

        }
    }

    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
        for (int i = 0; i < cloud->points.size(); i++)
            insert(cloud->points[i], i);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol) {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }


};

#endif /* KDTREE_H_ */


