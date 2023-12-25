
#include <utility>
#include <type_traits>
#include <limits>
#include <vector>

#include<benchmark/benchmark.h>



// <-- new implimentaion of the cmp less -->
template <typename _Tp, typename _Up>
constexpr bool cmp_less_new(_Tp __t, _Up __u) noexcept {
  if constexpr (sizeof(_Tp) < sizeof(long long) && sizeof(_Up) < sizeof(long long))
    return static_cast<long long>(__t) < static_cast<long long>(__u);
  else if constexpr (std::is_signed_v<_Tp> == std::is_signed_v<_Up>)
    return __t < __u;
  else if constexpr (std::is_signed_v<_Tp>)
    return __t < 0 ? true : std::make_unsigned_t<_Tp>(__t) < __u;
  else
    return __u < 0 ? false : __t < std::make_unsigned_t<_Up>(__u);
}


// <-- function to create vector of each type -->
template<typename T>
std::vector<T> make_vec(){
    const T min_value = std::numeric_limits<T>::min();
    const T max_value = std::numeric_limits<T>::max();
    // const T inc = max_value/10;
    std::vector<T> vec(7); //69,29,41
    vec[0] = min_value;
    vec[1] = 0;
    vec[2] = 1;
    vec[3] = 29;
    vec[4] = 41;
    vec[5] = 69;
    vec[6] = max_value;
    return vec;
}


// <-- all the int types -->
std::tuple<
    unsigned long long, long long, unsigned long, long, unsigned int, int,
    unsigned short, short, unsigned char, signed char> types;



// <-- testing  -->
// <-- the test implimentation by the new cmp less -->

template <typename T, typename U>
void test_cmp_less_new() {
    std::vector<T> tvec = make_vec<T>();
    std::vector<U> uvec = make_vec<U>();

    for(int i = 0 ; i < tvec.size() && i < uvec.size() ; i++){
        benchmark::DoNotOptimize(cmp_less_new(tvec[i],uvec[i]));
        benchmark::DoNotOptimize(cmp_less_new(tvec[i],uvec[uvec.size() - i -1]));
    }
}

template <class Ts,class... Us>
void test_new(std::tuple<Us ...>&){
    (test_cmp_less_new<Ts,Us>(), ...);
}

// <-- implentation for the std cmp less -->

template <typename T, typename U>
void test_cmp_less_std() {
    std::vector<T> tvec = make_vec<T>();
    std::vector<U> uvec = make_vec<U>();

    for(int i = 0 ; i < tvec.size() && i < uvec.size() ; i++){
        benchmark::DoNotOptimize(std::cmp_less(tvec[i],uvec[i]));
        benchmark::DoNotOptimize(std::cmp_less(tvec[i],uvec[uvec.size() - i -1]));
    }

}

template <class Ts,class... Us>
void test_std(std::tuple<Us ...>&){
    (test_cmp_less_std<Ts,Us>(), ...);
}

// <--- BM templater functions -->

template<class T>
static void BM_cmp_less_std(benchmark::State& state){
    for(auto _ : state){
        test_std<T>(types);
    }
}

template<class T>
static void BM_cmp_less_new(benchmark::State& state){
    for(auto _ : state){
        test_new<T>(types);
    }
}


BENCHMARK_TEMPLATE(BM_cmp_less_std, unsigned long long);
BENCHMARK_TEMPLATE(BM_cmp_less_new, unsigned long long);

BENCHMARK_TEMPLATE(BM_cmp_less_std, long long);
BENCHMARK_TEMPLATE(BM_cmp_less_new, long long);

BENCHMARK_TEMPLATE(BM_cmp_less_std, unsigned long);
BENCHMARK_TEMPLATE(BM_cmp_less_new, unsigned long);

BENCHMARK_TEMPLATE(BM_cmp_less_std, long);
BENCHMARK_TEMPLATE(BM_cmp_less_new, long);

BENCHMARK_TEMPLATE(BM_cmp_less_std, unsigned int);
BENCHMARK_TEMPLATE(BM_cmp_less_new, unsigned int);

BENCHMARK_TEMPLATE(BM_cmp_less_std, int);
BENCHMARK_TEMPLATE(BM_cmp_less_new, int);

BENCHMARK_TEMPLATE(BM_cmp_less_std, unsigned short);
BENCHMARK_TEMPLATE(BM_cmp_less_new, unsigned short);

BENCHMARK_TEMPLATE(BM_cmp_less_std, short);
BENCHMARK_TEMPLATE(BM_cmp_less_new, short);

BENCHMARK_TEMPLATE(BM_cmp_less_std, unsigned char);
BENCHMARK_TEMPLATE(BM_cmp_less_new, unsigned char);

BENCHMARK_TEMPLATE(BM_cmp_less_std, signed char);
BENCHMARK_TEMPLATE(BM_cmp_less_new, signed char);


BENCHMARK_MAIN();


// BENCHMARK(BM_cmp_less_new);