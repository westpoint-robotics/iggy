#include <nodelet/nodelet.h>

namespace igvc_image_pipeline
{
  
  class LineFilter : public nodelet::Nodelet
  {
    public:
      virtual void onInit();
  };
}
