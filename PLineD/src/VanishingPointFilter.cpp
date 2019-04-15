#include <../include/VanishingPointFilter.hpp>


namespace VP{
    void filterLines(std::vector<math::mathLine2d> src,std::vector<math::mathLine2d> &dst,double min_point2line_error, double max_point_cluster_error){
        std::vector<VPoint> VPs;
        VP::calculateVanishingPoint(src,VPs);

        std::vector<std::vector<Cand>> CP(VPs.size());
        buildClusterData(VPs,CP);

        Cand result;
        //for(auto &x: CP[0]) std::cout << "err: " << x.error << std::endl;

        bool valid = findClusterCenter(CP,result,src.size(),max_point_cluster_error,min_point2line_error);
        std::cout << "Valid: " << valid << std::endl;
        if(valid){
            for(uint i = 0; i < src.size(); i++){
                if(distance(src[i],result.p.p)<result.error){
                    dst.push_back(src[i]);
                }
            }
        }else{
            dst = src;
        }
    }
    VPoint calculateVanishingPoint(const math::mathLine2d &l1, const math::mathLine2d &l2){
        VPoint v;
        v.p.x = (l2.b-l1.b)/(l1.a-l2.a);
        v.p.y = l1.a*v.p.x+l1.b;
        return v;

    }
    void calculateVanishingPoint(const std::vector<math::mathLine2d> &src,std::vector<VPoint> &dst){
        for(uint i = 0; i < src.size(); i++){
            for(uint k = i+1; k < src.size(); k++){
                VPoint VP = calculateVanishingPoint(src[i],src[k]);
                VP.ID1 = i;
                VP.ID2 = k;
                dst.push_back(VP);
            }
        }
    }
    void buildClusterData(const std::vector<VPoint> &src, std::vector<std::vector<Cand>> &dst){
        dst = std::vector<std::vector<Cand>>(src.size());
        for(uint i = 0; i < src.size(); i++){
            for(uint k = i+1; k< src.size(); k++){
                double d = math::distance(src[i].p,src[k].p);
                dst[i].push_back({src[i],d});
                dst[k].push_back({src[k],d});
            }
            std::sort(dst[i].begin(),dst[i].end(),candComp);
        }
    }
    bool findClusterCenter(std::vector<std::vector<Cand>> src, Cand &dst,uint error_check_index, double max_error, double min_error){
        double min_avg = max_error*2;
        int index = -1;
        for(uint i = 0; i < src.size() ; i++){
            if(src[i][error_check_index].error < max_error){
                double max_e = src[i][error_check_index].error*2;
                double sum = 0;
                uint k;
                for(k=0; k < src[i].size(); k++){
                    sum+=src[i][k].error;
                    if(src[i][k].error > max_e) break; 
                }
                src[i].erase(src[i].begin()+k,src[i].end());
                double avg = sum/k;
                if(avg < min_avg){
                    min_avg = avg;
                    index = i;
                } 
            }
        }
        std::cout << "Index: " << index << std::endl;
        if(index != -1){
            dst = src[index].back();
            if(dst.error < min_error) dst.error = min_error;
        }      
        return (index != -1);
    }
}