"""

author: Bran Yuan

"""

import scipy.spatial

class KDTree:
    """
    基于KDTree的最近点查询树
    """

    def __init__(self, data):
        # KDTree储存
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        搜索，k=1返回最近点下标，k=2返回range(k)点下标，k=[]返回列表距离下标
        inp: 单点或者点表
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                # 查询距离点i最近点距离和下标
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        """
        搜索距离小于半径r的所有点列表
        """
        index = self.tree.query_ball_point(inp, r)
        return index



if __name__ == '__main__':
    pass
