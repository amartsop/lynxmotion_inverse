#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

template <class M>

class SignalProcessing{
    public:
        // Contructor
        SignalProcessing();
        
        // Main methods
        static M map(M x, M in_min, M in_max, M out_min, M out_max);
        static M clipping(M x, M x_l, M x_u);
        static M saturation(M x, M x_lim);
        static M deadbandRecovery(M x, M x_l, M x_u);
        static M expMovingAverage(M x, M x_prev, M alhpa);
        static M sgn(M x);
         static M rad2Deg(M x);
        static bool inRange(M x, M x_l, M x_u);
       
    private:
        static M absM(M x);
};

template <class M>
SignalProcessing<M>::SignalProcessing(){}

template <class M>
M SignalProcessing<M>::map(M x, M in_min, M in_max, M out_min, M out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <class M>
M SignalProcessing<M>::clipping(M x, M x_l, M x_u){    
    if (x > x_u){
        x = x_u; 
        return x;
    }
    else if (x < x_l){
        x = x_l;
        return x;
    }
    else{
        return x;
    }
}

template <class M>
M SignalProcessing<M>::deadbandRecovery(M x, M x_l, M x_u)
{
    return ((x >= x_u) | ( x <= x_l )) ? (x) : ((M) 0);
}

template <class M> 
M SignalProcessing<M>::sgn(M x) {
    return (M(0) < x) - (x < M(0));
}

template <class M>
M SignalProcessing<M>::saturation(M x, M x_lim){
    
    if((x + absM(x_lim)) < (M) 0){
        return (M) -1;
    }
    else if((x - absM(x_lim)) > (M) 0){
    
        return (M) 1;    
    }
    else{
        return (M) x/x_lim;
    }
}

template <class M>
M SignalProcessing<M>::expMovingAverage(M x, M x_prev, M alhpa){
    M y; y = alhpa*x + (1 - alhpa)*x_prev; return y;
}

template <class M>
M SignalProcessing<M>::absM(M x){
    M y; (x >= (M) 0) ? (y = x) : (y = (-1) * x); return y;
}

template <class M>
bool SignalProcessing<M>::inRange(M x, M x_l, M x_u)
{
    return ((x - x_u) * (x - x_l) <= 0); 
}

template <class M>
M SignalProcessing<M>::rad2Deg(M x)
{
    M radCoef =  (M) (180.0 / PI);
    return (x * radCoef);
}


#endif