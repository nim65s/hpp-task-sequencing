//
// Created by Jeff on 2019/1/2 0002.
//
#include "isodata.h"

/**
 * initializes the clustering by randomly selecting nc centers
 */
void isodata::init_clusters() {
  // randomizer initialisation
  std::default_random_engine rand(static_cast<unsigned int>(time(nullptr)));
  std::uniform_int_distribution<unsigned> rnd(0, rows-1);
  std::unordered_set<unsigned> ids;
  while (ids.size() < _nc)
    {
      // ranom idx of a point to become a centroid
      auto id = rnd(rand);
      // to avoid selecting a centroid twice
      bool flag = false;
      for (auto &item : ids)
        {
	  if (get_distance(data.row(item), data.row(id), k_task) == 0.0)
	    flag = true;
        }
      if (!flag)
	ids.emplace(id);
    }
  // store the obtained clusters
  for (auto &id : ids)
    clusters.emplace_back(Cluster(data.row(id)));
}

/**
 * looks for the closest centroid to a point to put it in a cluster
 * @p_index id of the point in the data
 * @ignore number of the cluster to ignore (mainly used for cluster deletion)
 * @return id of the cluster and distance
 */
std::pair<int, double> isodata::get_nearest_cluster(int p_index, int ignore = -1) {
  if (ignore != -1 && int(clusters.size()) == 1)
    std::cout << WARN_CLUSTER_SIZE_SMALL << std::endl;
  int c_index = 0;
  double dist(get_distance(data.row(p_index), clusters[c_index].center, k_task));
  // look for the closest centroid
  for (int i = 1; unsigned(i) < clusters.size(); ++i) {
    if (ignore != -1 && i == ignore)
      continue;
    auto d = get_distance(data.row(p_index), clusters[i].center, k_task);
    // if closer : update closest one
    if (d < dist)
      {
	dist = d;
	c_index = i;
      }
  }
  return {c_index, dist};
}

/**
 * reallocation of the points according to the distance to the centroids
 */
void isodata::re_assign() {
  // empty all clusters
  for (auto &cluster : clusters) {
    cluster.clear_ids();
  }
  // reassign points to the closest centroids
  for (unsigned i = 0; i < rows; ++i)
    {
      auto&& res = get_nearest_cluster(i);
      clusters[res.first].add_point(i);
    }
}

/**
 * tests if there are less than tn points in a cluster
 * if it is the case : cluster deleted
 */
void isodata::check_tn(bool& flag) {
  // chech if size<tn
  for (unsigned i = 0; i < clusters.size(); ++i) {
    if (clusters[i].ids.size() < _tn){
      clusters[i].ids.clear(); // if size<tn the cluster is emptied
      flag = true;
    }
  }
  // delete emptied clusters
  for (auto it = clusters.begin(); it != clusters.end();)
    {
      if (it->ids.empty())
	it = clusters.erase(it);
      else
	++it;
    }
}

/**
 * updates the centroid of each cluster
 */
void isodata::update_centers() {
  for (auto &cluster : clusters)
    {
      update_center(cluster);
    }
}

/**
 * updates the centroid of a given cluster
 */
void isodata::update_center(Cluster& cluster) {
  Eigen::ArrayXd sum(cols);
  sum.setZero();
  for (auto &index : cluster.ids)
    {
      Eigen::ArrayXd line = data.row(index);
      sum += line;
    }
  cluster.center = sum/ static_cast<double>(cluster.ids.size());
}


/**
 * updates the variance of each cluster
 */
void isodata::update_sigmas() {
  for ( auto& cluster : clusters)
    update_sigma(cluster);
}

/**
 * updates the variance of a given cluster
 */
void isodata::update_sigma(Cluster& cluster) {
  Eigen::ArrayXd sum(cols);
  sum.setZero();
  for (auto& id : cluster.ids) {
    sum += Eigen::pow(cluster.center - data(id), 2);
  }
  sum /= static_cast<double>(cluster.ids.size());
  cluster.sigma = Eigen::sqrt(sum);
}

/**
 * updates mean distances (global and of every cluster)
 */
void isodata::update_meandis() {
  Cluster::allMeanDis = 0;
  for (auto &&cluster : clusters) {
    double dis(0.0);
    for (auto &id : cluster.ids) {
      dis += get_distance(data.row(id), cluster.center, k_task);
    }
    Cluster::allMeanDis += dis;
    cluster.innerMeanDis = dis/double(cluster.ids.size());
  }
  Cluster::allMeanDis /= rows;
}

/**
 * splits cluster c_index according to the variance
 */
void isodata::split(const int &c_index) {
  auto& cluster = clusters[c_index];
  // retrieving the index of the biggest variance
  EIGEN_DEFAULT_DENSE_INDEX_TYPE iter;
  cluster.sigma.maxCoeff(&iter);
  long int pos(iter); // converting it to an int
  Cluster newcluster(cluster.center);
  // splitting
  newcluster.center[pos] -= alpha*cluster.center[pos];
  cluster.center[pos] += alpha*cluster.center[pos];
  for (const auto &id : cluster.ids)
    {
      auto d1 = get_distance(data.row(id), cluster.center, k_task);
      auto d2 = get_distance(data.row(id), newcluster.center, k_task);
      if (d2 < d1)
	newcluster.ids.emplace(id);
    }
  std::unordered_set<unsigned> dids;
  set_difference(cluster.ids.begin(), cluster.ids.end(),
		 newcluster.ids.begin(), newcluster.ids.end(),
		 inserter(dids, dids.begin()));
  swap(dids, cluster.ids);
  // updates the parameters and saves the new clusters
  update_center(newcluster);
  update_sigma(newcluster);
  update_center(cluster);
  update_sigma(cluster);
  clusters.emplace_back(newcluster);
}

/**
 * checks if a merge is necessary
 * if one is, it is done
 */
void isodata::check_merge() {
  // personalized data type
  typedef std::pair<std::pair<unsigned, unsigned>, double> UNIT;
  //1 computation of the distance between two centroids, keeping those closer than tc
  //  and ordering by ascending distance
  std::vector<UNIT> uvec;
  for (unsigned i = 0; i < clusters.size(); ++i) {
    for (unsigned j = i+1; j < clusters.size(); ++j) {
      auto dis = get_distance(clusters[i].center, clusters[j].center, k_task);
      if (dis < _tc)
	uvec.emplace_back(UNIT({i,j}, dis));
    }
  }
  sort(uvec.begin(), uvec.end(), [](UNIT& left, UNIT& right){ return left.second < right.second;});
  //2 merging operation
  std::unordered_set<unsigned> clusterIds{};
  unsigned cnt(0);
  for (const auto &unit : uvec) {
    auto& cids = unit.first;
    if (clusterIds.find(cids.first) == clusterIds.end() &&
	clusterIds.find(cids.second) == clusterIds.end())
      {
	merge(cids.first, cids.second);
	clusterIds.emplace(cids.first);
	clusterIds.emplace(cids.second);
	++cnt;
      }
    // detects exceeding of max nb of mergings 
    if (cnt > _ns)
      break;
  }
  //3 clear merged clusters
  for (auto iter = clusters.begin(); iter != clusters.end();)
    {
      if (iter->ids.empty())
	iter = clusters.erase(iter);
      else
	iter++;
    }
}

/**
 * merges clusters id1 and id2
 */
void isodata::merge(const int &id1, const int &id2) {
  auto &c1 = clusters[id1];
  auto &c2 = clusters[id2];
  // new centroid
  c1.center = (c1.center*c1.ids.size() + c2.center*c2.ids.size())/(c1.ids.size()+c2.ids.size());
  c1.ids.insert(c2.ids.begin(), c2.ids.end());
  c2.ids.clear(); // remove cluster id2
}

/**
 * writes the clusters into a vector
 */
std::vector<ResultCluster> isodata::resultFormatting() const{
  std::vector<ResultCluster> res;
  // for every cluster
  for (std::deque<Cluster>::const_iterator clsIt=clusters.begin(); clsIt!=clusters.end(); ++clsIt)
    {
      ResultCluster resCls;
      // retrieving the centroid
      resCls.centroid = (*clsIt).center;
      // retrieving the points
      resCls.points.resize((*clsIt).ids.size(), cols);
      int i = 0;
      for (std::unordered_set<unsigned>::const_iterator it=(*clsIt).ids.begin(); it!=(*clsIt).ids.end(); ++it){
	for (unsigned j=0; j<cols; j++)
	  resCls.points(i,j) = data(*it,j);
	i++;
      }
      // adding the cluster to the result
      res.push_back(resCls);
    }
  return res;
}
