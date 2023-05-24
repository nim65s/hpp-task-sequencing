//
// Created by Jeff on 2019/1/2 0002.
//
#include "isodata.h"

/**
 * initializes the clustering by randomly selecting nc centers
 */
void isodata::init_clusters() {
  // selects a random id
  std::default_random_engine rand(static_cast<unsigned int>(time(nullptr)));
  std::uniform_int_distribution<unsigned> rnd(0, row-1);
  std::unordered_set<unsigned> ids;
  while (ids.size() < _nc)
    {
      auto id = rnd(rand);
      // to avoid selecting a centroid twice
      bool flag = false;
      for (auto &item : ids)
        {
	  if (get_distance(data.row(item), data.row(id)) == 0.0)
	    flag = true;
        }
      if (!flag)
	ids.emplace(id);
    }
  // initial clustering
  for (auto &id : ids)
    clusters.emplace_back(Cluster(data.row(id)));
}

/**
 * looks for the closest centroid to a point to put it in a cluster
 * @p_index id of the point in the data
 * @ignore number of the cluster is ignored (mainly used for cluster deletion)
 * @return id of the cluster and distance
 */
std::pair<int, double> isodata::get_nearest_cluster(int p_index, int ignore = -1) {
  if (ignore != -1 && clusters.size() == 1)
    std::cout << WARN_CLUSTER_SIZE_SMALL << std::endl;
  int c_index = 0;
  double dis(get_distance(data.row(p_index), clusters[c_index].center));
  for (int i = 1; i < clusters.size(); ++i) {
    if (ignore != -1 && i == ignore)
      continue;
    auto d = get_distance(data.row(p_index), clusters[i].center);
    if (d < dis)
      {
	dis = d;
	c_index = i;
      }
  }
  return {c_index, dis};
}
/**
 * looks for the closest centroid to a point to put it in a cluster
 * @param p_index id of the point in the data
 * @param cluster_ids set of ids of the clusters not to be taken into account
 * @return id of the cluster and distance
 */
std::pair<int, double> isodata::get_nearest_cluster(int p_index, std::unordered_set<unsigned> &cluster_ids) {
  // select the first cluster that is not in cluster_ids
  int c_index = 0;
  while (cluster_ids.find(static_cast<const unsigned int &>(c_index)) != cluster_ids.end())
    ++c_index;
  // initial distance
  double dis(get_distance(data.row(p_index), clusters[c_index].center));
  for (int i = c_index+1;
       i < clusters.size() && (cluster_ids.find(static_cast<const unsigned int &>(i)) == cluster_ids.end());
       ++i)
    {
      auto d = get_distance(data.row(p_index), clusters[i].center);
      if (d < dis)
        {
	  dis = d;
	  c_index = i;
        }
    }
  return {c_index, dis};
}

/**
 * reallocation of the points according to the distance to the centroids
 */
void isodata::re_assign() {
  for (auto &cluster : clusters) {
    cluster.clear_ids();
  }
  for (int i = 0; i < row; ++i)
    {
      auto&& res = get_nearest_cluster(i);
      clusters[res.first].add_point(i);
    }
}

// tests if there are less than tn points in a cluster
// if it is the case, the cluster is deleted
void isodata::check_tn() {
  std::unordered_set<unsigned> to_erase;
  for (int i = 0; i < clusters.size(); ++i) {
    if (clusters[i].ids.size() >= _tn)
      continue;
    to_erase.emplace(i);
    auto &indexs = clusters[i].ids;
    for (unsigned int index : indexs)
      {
	auto &&res = get_nearest_cluster(index, to_erase);
	clusters[res.first].add_point(index);
      }
    clusters[i].ids.clear();

  }
  for (auto it = clusters.begin(); it != clusters.end();)
    {
      if (it->ids.empty())
	it = clusters.erase(it);
      else
	++it;
    }
}

// updates the center of each cluster
void isodata::update_centers() {
  for (auto &cluster : clusters)
    {
      update_center(cluster);
    }
}

// updates the center of a given cluster
void isodata::update_center(Cluster& cluster) {
  Eigen::ArrayXd sum(col);
  sum.setZero();
  for (auto &index : cluster.ids)
    {
      Eigen::ArrayXd line = data.row(index);
      sum += line;
    }
  cluster.center = sum/ static_cast<double>(cluster.ids.size());
}


// updates the variance of each cluster
void isodata::update_sigmas() {
  for ( auto& cluster : clusters)
    update_sigma(cluster);
}

// updates the variance of a cluster
void isodata::update_sigma(Cluster& cluster) {
  Eigen::ArrayXd sum(col);
  sum.setZero();
  for (auto& id : cluster.ids) {
    sum += Eigen::pow(cluster.center - data(id), 2);
  }
  cluster.sigma = Eigen::sqrt(sum);
}

// updates all mean distances
void isodata::update_meandis() {
  Cluster::allMeanDis = 0;
  for (auto &&cluster : clusters) {
    double dis(0.0);
    for (auto &id : cluster.ids) {
      dis += get_distance(data.row(id), cluster.center);
    }
    Cluster::allMeanDis += dis;
    cluster.innerMeanDis = dis/cluster.ids.size();
  }
  Cluster::allMeanDis /= row;
}


// checks if a cluster has to be split
// if it is the case, the splitting is done
void isodata::check_split() {
  update_sigmas();
  while (true)
    {
      bool flag = false;
      for (unsigned j = 0; j < clusters.size(); ++j) {
	auto& cluster = clusters[j];
	for (int i = 0; i < col; ++i)
	  {
	    if (cluster.innerMeanDis > Cluster::allMeanDis)
	      {
		if (cluster.sigma[i] > _te && (cluster.ids.size() > 2 * _tn + 1 || clusters.size() < _c / 2))
		  {
		    flag = true;
		    split(j);
		  }
	      }
	    else
	      {
		if (cluster.sigma[i] > _te && clusters.size() < _c / 2)
		  {
		    flag = true;
		    split(j);
		  }
	      }
	  }
      }
      if (!flag)
	break;
      else
	update_meandis();
    }
}

// splits cluster c_index
void isodata::split(const int &c_index) {
  // selects where to split according to the variance
  auto& cluster = clusters[c_index];
  // retrieving the index of the biggest variance
  EIGEN_DEFAULT_DENSE_INDEX_TYPE iter;
  cluster.sigma.maxCoeff(&iter);
  int pos(iter); // converting it to an int
  Cluster newcluster(cluster.center);
  // splitting
  newcluster.center[pos] -= alpha*cluster.center[pos];
  cluster.center[pos] += alpha*cluster.center[pos];
  for (const auto &id : cluster.ids)
    {
      auto d1 = get_distance(data.row(id), cluster.center);
      auto d2 = get_distance(data.row(id), newcluster.center);
      if (d2 < d1)
	newcluster.ids.emplace(id);
    }
  std::unordered_set<unsigned> dids;
  set_difference(cluster.ids.begin(), cluster.ids.end(),
		 newcluster.ids.begin(), newcluster.ids.end(),
		 inserter(dids, dids.begin()));
  swap(dids, cluster.ids);
  // update of the parameters and saving of the new clusters
  update_center(newcluster);
  update_sigma(newcluster);
  update_center(cluster);
  update_sigma(cluster);
  clusters.emplace_back(newcluster);
}

// checks if a merge is necessary
// if one is, it is done
void isodata::check_merge() {
  // personalized data type
  typedef std::pair<std::pair<unsigned, unsigned>, double> UNIT;
  //1 computation of the distance between two centers, keeping those closer than tc and ordering by ascending distance
  std::vector<UNIT> uvec;
  for (unsigned i = 0; i < clusters.size(); ++i) {
    for (unsigned j = i+1; j < clusters.size(); ++j) {
      auto dis = get_distance(clusters[i].center, clusters[j].center);
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
    auto& cdis = unit.second;
    if (clusterIds.find(cids.first) == clusterIds.end() &&
	clusterIds.find(cids.second) == clusterIds.end())
      {
	merge(cids.first, cids.second);
	clusterIds.emplace(cids.first);
	clusterIds.emplace(cids.second);
      }
    // detects exceeding of max nb of mergings 
    if (++cnt > _ns)
      break;
  }
  //3 clear the merged clusters
  for (auto iter = clusters.begin(); iter != clusters.end();)
    {
      if (iter->ids.empty())
	iter = clusters.erase(iter);
      else
	iter++;
    }
}

// merges clusters id1 and id2
void isodata::merge(const int &id1, const int &id2) {
  auto &c1 = clusters[id1];
  auto &c2 = clusters[id2];
  c1.center = (c1.center* c1.ids.size() + c2.center*c2.ids.size())/(c1.ids.size()+c2.ids.size());
  c2.ids.clear();
}

// select the next appropriate operation
void isodata::switch_method(const int& index) {
  if (index == _ns-1)
    {
      _tc = 0;
      check_merge();
    }
  else if (clusters.size() <= _c/2)
      check_split();
  else if (index % 2 == 0 || clusters.size() >= 2*_c)
      check_merge();
  else
      check_split();
}

// writes the clusters into a vector
std::vector<ResultCluster> isodata::resultFormatting() const{
  std::vector<ResultCluster> res;
  for (std::deque<Cluster>::const_iterator clsIt=clusters.begin(); clsIt!=clusters.end(); ++clsIt)
    {
      ResultCluster resCls;
      // retrieving the centroid
      resCls.centroid = (*clsIt).center;
      // retrieving the points
      resCls.points.resize((*clsIt).ids.size(), col);
      int i = 0;
      for (std::unordered_set<unsigned>::const_iterator it=(*clsIt).ids.begin(); it!=(*clsIt).ids.end(); ++it){
	for (int j=0; j<col; j++)
	  resCls.points(i,j) = data(*it,j);
	i++;
      }
      // adding the cluster to the result
      res.push_back(resCls);
    }
  return res;
}
