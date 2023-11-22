#ifndef AS2_OUTPUT_HPP_10348241798
#define AS2_OUTPUT_HPP_10348241798

namespace as2_platform_dji_psdk
{

template<typename PublisherDefinition>
class Output
{
public:
  using Msg_t = typename PublisherDefinition::Msg_t;
  using PublisherSharedPtr = typename rclcpp::Publisher<Msg_t>::SharedPtr;

  void init(rclcpp::Node * node)
  {
    publisher_ = node->create_publisher<Msg_t>(PublisherDefinition::name, 10);
  }

  void publish() const {publisher_->publish(state_variable);}

  Msg_t * operator->() {return &state_variable;}

protected:
  Msg_t state_variable;
  PublisherSharedPtr publisher_;
};

}  // namespace as2_platform_dji_psdk

#endif
