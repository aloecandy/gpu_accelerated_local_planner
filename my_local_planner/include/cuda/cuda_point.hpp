#ifndef CUDA_POINT_HPP
#define CUDA_POINT_HPP

class CuPoint{
    public:
        double x;
        double y;
        double z;
};

#endif

/* need to pass two variables (addr,size)

int cu_footprint_size=footprint_spec _.size();
CuPoint* cu_footprint_spec_ = new CuPoint[cu_footprint_size];

for(int i=0;i<cu_footprint_size;i++){
    cu_footprint_spec_[i].x=footprint_spec_[i].x;
    cu_footprint_spec_[i].y=footprint_spec_[i].y;
    cu_footprint_spec_[i].z=footprint_spec_[i].z;
}
printf("test\n");
for(int i=0;i<cu_footprint_size;i++){
    if(cu_footprint_spec_[i].x!=footprint_spec_[i].x)   printf("n.m.\n");
    if(cu_footprint_spec_[i].y!=footprint_spec_[i].y)   printf("n.m.\n");
    if(cu_footprint_spec_[i].z!=footprint_spec_[i].z)   printf("n.m.\n");
}

*/