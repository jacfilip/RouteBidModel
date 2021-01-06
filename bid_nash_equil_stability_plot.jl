using Pkg
using DataFrames
using Random
using Distributions
using CSV
using LaTeXStrings
using Plots

pyplot()

using Revise
Pkg.activate(".")
using RouteBidModel

Random.seed!(1);

#simulation of bidding behavior
N_MAX=200
p2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq = NashEq(p2)

costneq_pa=sum(cost(p2,neq))/p2.N_MAX
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 300, optimizebid)

CSV.write(raw".\results\log_stepbifurcate.csv", log_step) #seed = 1
CSV.write(raw".\results\log_agbifurcate.csv", log_ag)


Random.seed!(0);
p2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

solve_middle_payment(p2)

bidm, log_ag_m,log_step_m = play_global_nash(p2, p2.ct, 220, optimizebid_middle)
CSV.write(raw".\results\log_step_m.csv", log_step_m) #seed = 0
CSV.write(raw".\results\log_ag_m.csv", log_ag_m)


neq = NashEq(p2)
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 220, optimizebid) #seed = 0
CSV.write(raw".\results\log_stepcalmseed0.csv", log_step)
CSV.write(raw".\results\log_agcalmseed0.csv", log_ag)



function doplot(log_step::DataFrame, neq::NashEq; kw...)
  log_step.costneq_pa = (log_step.costneq1.*log_step.n0.+log_step.costneq2.*log_step.n1)./(log_step.n0+log_step.n1)
  @assert minimum(log_step.costneq_pa) ≈ maximum(log_step.costneq_pa)
  plot(1:nrow(log_step),log_step.cost_real_avg./log_step.costneq_pa, label="Total costs: "*L"C^*_{TOT} / C^\dagger_{TOT}",
   linewidth=4,
   size=(720,460),
   linecolor=:black,
   yformatter = yi -> "\$$(round(Int,100yi))\$ %",
   xlabel = "Simulation steps",
   ylabel = "Value relative to Nash Equilibrium";
   kw...
   )
  plot!(1:nrow(log_step),(log_step.cost1.+log_step.pmnt1) ./log_step.costneq1, label=raw"Net costs route $s_0$: "*L"((\mathbf{C}^*-\mathbf{p})^T(\mathbf{1}-\mathbf{x}^*)) /((\mathbf{C}^\dagger)^T(\mathbf{1}-\mathbf{x}^*))", linewidth=2,linecolor=:red)
  plot!(1:nrow(log_step),(log_step.cost2.+log_step.pmnt2)./log_step.costneq2, label=raw"Net costs route $s_1$: "*L"((\mathbf{C}^*-\mathbf{p})^T\mathbf{x}^*) /((\mathbf{C}^\dagger)^T\mathbf{x}^*)", linewidth=2,linecolor=:green)
  plot!(1:nrow(log_step),log_step.n0./neq.n0, label=raw"Cars on route $s_0$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:red)
  plot!(1:nrow(log_step),log_step.n1./neq.n1, label=raw"Cars on route $s_1$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:green)
  plot!(1:nrow(log_step),log_step.t_1./log_step.neq_t, label=raw"Travel time route $s_0$: "*L"t_0^*/t^\dagger",linestyle=:dot,linecolor=:red)
  plot!(1:nrow(log_step),log_step.t_2./log_step.neq_t, label=raw"Travel time route $s_1$: "*L"t_1^*/t^\dagger",linestyle=:dot,linecolor=:green)

end

log_step = CSV.read(raw".\results\log_stepbifurcate.csv")
doplot(log_step[1:220,:], neq)
savefig("bifurcate.png")
savefig("bifurcate.pdf")#seed=1


log_step_m = CSV.read(raw".\results\log_step_m.csv")
doplot(log_step_m[1:220,:], neq; ) #ylim=(-1.7,3.3)  # seed =0
#ylim=(-1.1,8.1),yticks=-1:7
savefig("degenerate.pdf")
savefig("degenerate.png")

log_step_calm = CSV.read(raw".\results\log_stepcalmseed0.csv")
doplot(log_step_calm[1:210,:],neq;ylim=(0.6,1.6)) #seed 0
savefig("calm_dyn.pdf")
savefig("calm_dyn.png")

#heterogenity sensitivity

function lognorm_params(m, std_dev)
    v = std_dev^2
    ϕ = sqrt(v + m^2);
    μ  = log(m^2/ϕ)
    σ = sqrt(log(ϕ^2/m^2))
    (μ, σ)
end

dist_ln = Distributions.LogNormal(3,1)

m=mean(dist_ln)
std_dev=std(dist_ln)

params = lognorm_params.(m, (0.4:0.05:1.6).*std_dev)

function simulate_multiple_distributions(ln_params; N_MAX=200)
  Random.seed!(0);
  sensitivity = DataFrame(sigma = Float64[], init_tot_cost = Float64[], final_tot_cost = Float64[],
    init_s0_cost = Float64[], final_s0_cost = Float64[],
    init_s1_cost = Float64[], final_s1_cost = Float64[],
    init_n0 = Float64[], final_n0 = Float64[],
    init_n1 = Float64[], final_n1 = Float64[],
    convergence_iters = Int64[], cost_swaps = Int64[])

  for p in ln_params
    model_p = BidModelParams(N_MAX=N_MAX, N=(150,100),
      ct=rand(LogNormal(p[1],p[2]),N_MAX), # $/h
      d=(10.0, 10.0), # km
      v_max=(60, 60), # km/h
      v_min=(5, 5), # km/h
      cf=0
    )

    neq = NashEq(model_p)
    bidm, log_ag, log_step = play_global_nash(model_p, model_p.ct, 300, optimizebid)
    log_step.costneq_pa = (log_step.costneq1.*log_step.n0.+log_step.costneq2.*log_step.n1)./(log_step.n0+log_step.n1)
    tot_cost = log_step.cost_real_avg./log_step.costneq_pa

    net_cost0 = (log_step.cost1.+log_step.pmnt1) ./log_step.costneq1
    net_cost1 = (log_step.cost2.+log_step.pmnt2) ./log_step.costneq2

    cost = 0
    cnt_conv = 0
    cnt_swap = 0
    last_sign = 0
    cnt_sign = 0
    for i in 1:length(tot_cost)
      if cost != tot_cost[i] cnt_conv += 1 end
      cost = tot_cost[i]

      if sign(net_cost0[i] - net_cost1[i]) != last_sign cnt_sign += 1 end
      last_sign = sign(net_cost0[i] - net_cost1[i])
    end

    push!(sensitivity, Dict(:sigma => p[2], :init_tot_cost => tot_cost[1], :final_tot_cost => tot_cost[end],
                          :init_s0_cost => net_cost0[1],  :final_s0_cost => net_cost0[end],
                          :init_s1_cost => net_cost1[1],  :final_s1_cost => net_cost1[end],
                          :init_n0 => (log_step.n0./neq.n0)[1], :final_n0 => (log_step.n0./neq.n0)[end],
                          :init_n1 => (log_step.n1./neq.n1)[1], :final_n1 => (log_step.n1./neq.n1)[end],
                          :convergence_iters => cnt_conv, :cost_swaps => cnt_sign)
                          )

  end
  return  sensitivity
end

sensitivity = simulate_multiple_distributions(params)
CSV.write(raw".\results\sensitivity.csv", sensitivity)

function plot_sensitivity(data::DataFrame)
  dev = std.(LogNormal.(3, data.sigma))

  plot(dev, data.init_tot_cost, label="Total cost on first iteration",
   linewidth=4,
   size=(720,460),
   linecolor=:black,
   yformatter = yi -> "\$$(round(Int,100yi))\$ %",
   xlabel = raw"Cost of time standard deviation, $/h",
   ylabel = "Value relative to Nash Equilibrium"
   )
  plot!(dev, data.final_tot_cost, label=raw"Total cost after reaching equilibrium", linecolor=:black, linestyle=:dash,)
  plot!(dev, data.init_s0_cost, label=raw"$s_0$ cost on first iteration", linewidth=2,linecolor=:green)
  plot!(dev, data.init_s1_cost, label=raw"$s_1$ cost on first iteration", linewidth=2,linecolor=:red)
  plot!(dev, data.final_s0_cost, label=raw"$s_0$ cost after reaching equilibrium",  linestyle=:dot,linecolor=:green)
  plot!(dev, data.final_s1_cost, label=raw"$s_1$ cost after reaching equilibrium",  linestyle=:dot,linecolor=:red)
  #plot!(data.sigma, data.convergence_iters, label=raw"Iterations to convergence", linestyle=:dot,linecolor=:blue, axis = :right)
end

plot_sensitivity(sensitivity)
savefig("sensitivity.png")
