//
// Created by NikoohematS on 1-9-2017.
//

#include <iostream>
#include <string>
#include "LaserPoints.h"
#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <boost/unordered_map.hpp>

bool compare_lp_segmenttag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(SegmentNumberTag) < lp2.begin()->Attribute(SegmentNumberTag);
}

bool compare_lp_labeltag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(LabelTag) < lp2.begin()->Attribute(LabelTag);
}

bool compare_lp_size(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.size() > lp2.size();  /// descending
}

bool compare_lp_label2tag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(Label2Tag) < lp2.begin()->Attribute(Label2Tag) ;
}

/// collect laserpoints per similar_tags and return a vector of them,
std::vector<LaserPoints> PartitionLpByTag(LaserPoints const& lp, LaserPointTag int_tag, const char* output= nullptr) {

    //printf("Partitioning points by tag... \n");
    /// if you don't have boost you can use std:unordered_multimap for newer compilers e.g. 2014
    typedef boost::unordered::unordered_multimap<int, LaserPoint> UnorderTaggedPoints;
    UnorderTaggedPoints umap;

    for (auto&& p : lp) {
        /// insert laser points and its key (tag number) as a pair to the multiumap
        umap.emplace(p.Attribute(int_tag), p);
    }

    //cout << "Umap size:" << umap.size() << endl; // debug

    std::vector<LaserPoints> result;
    int total_points=0;

    auto umap_begin = umap.begin();
    auto umap_end = umap.end();

    while (umap_begin != umap_end) {

        /// range is the iterator to pair of multimap objects
        auto range = umap.equal_range(umap_begin->first);

        LaserPoints tmp;
        tmp.reserve(std::distance(range.first, range.second));

        /// loop through the iterators of range and collect points with similar tag number
        for (auto i = range.first; i != range.second; ++i) {
            //std::move(i->second);
            boost::move(i->second);
            tmp.push_back(i->second);
        }
        total_points += tmp.size();
        result.push_back(std::move(tmp));
        umap_begin = range.second;
    }

    /// if there is output directory, write the tag partitions to the disk
    if (output){
        printf("Write partitions based on <%s> to the disk ... \n", AttributeName(int_tag, true));
        for (auto it=result.begin(); it != result.end(); ++it){
            int tag_nr = (it->begin())->Attribute(int_tag);

            if ((*it).size() > 2){

                string tmp_file;
                std::stringstream sstm;
                sstm << output << "/" << tag_nr << ".laser";
                tmp_file = sstm.str();
                (*it).Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
    }

    //cout << "Total number of partitioned points: " << total_points << endl;
    return result;
};