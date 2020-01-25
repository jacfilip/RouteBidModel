using Revise
using Pkg
Pkg.activate(".")
using RouteBidModel
using DataFrames
using Random
using Distributions
Random.seed!(0);

N_MAX = 30
p = BidModelParams(N_MAX=N_MAX,N=[20,20],ct=rand(LogNormal(3.2,0.20),N_MAX)./3600 )

ne = solve_nash_time(p)
e_cost = sum(cost(p, ne))

t = solve_travel_jump(p)
sum(t.cost)

opt = solve_travel(p)
sum(opt.cost)
sum(opt.cost_real)


Random.seed!(0);
bid6 = minimum(p.ct) .+ rand(length(p.ct))./10 .* p.ct
res6 = play_nash(p,bid6; N_STEPS=120)
sol6 = solve_travel(p,res6.bid)
sum(sol6.cost)
sum(sol6.cost_real)

pp = solve_optimal_payment(p,res6.bid)
pays = pp.payments
sum(pays)


sol6_ = solve_travel_jump(p,res6.bid)
sum(sol6_.cost)

d6=DataFrame(x=sol6.x,p=p.ct, bid=res6.bid)
display(sort(d6,:p))

res6


Random.seed!(0);
bid12 = 12 .+ rand(length(p.ct))./100
res12 = play_nash(p,bid12; N_STEPS=200)
sol12 = solve_travel(p,res12.bid)
sum(sol12.cost)
d12=DataFrame(x=sol12.x,p=p.ct, bid=res12.bid)
display(sort(d12,:bid))

Random.seed!(0);
bid_rand = 5 .+ rand(length(p.ct)) .* 10
res_rand = play_nash(p,bid_rand; N_STEPS=200)
sol_rand = solve_travel(p,res_rand.bid)
sum(sol_rand.cost)
d_rand=DataFrame(x=sol_rand.x,p=p.ct, bid=res_rand.bid)
display(sort(d_rand,:p))
