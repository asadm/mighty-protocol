#include "cpp/mighty_protocol.h"
#include <cassert>
int main(){
 using namespace mighty_protocol;
 auto p=build_image_drop_payload(123,"preview",19);
 uint64_t ts=0,count=0;std::string channel;
 assert(decode_image_drop_payload(p,ts,channel,count));
 assert(ts==123&&count==19&&channel=="preview"&&p.size()==25&&p[8]==1);
 p.pop_back();assert(!decode_image_drop_payload(p,ts,channel,count));
}
