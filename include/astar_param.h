

class ExParam
{
public:
    static ExParam& Instance()
    {
        static ExParam instance;
        return instance;
    }

private:
    ExParam()
    {
    }

    virtual ~ExParam()
    {
    }

public:
    int succ_size_;
    int forward_size_;
    int backward_size_;

    // static const int size = 30;

    int delta_x_[30];
    int delta_y_[30];
    int delta_theta_[30];
};
