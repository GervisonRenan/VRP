module UnPM

struct instance_VRP

    n::Int64
    m::Int64
    p::Int64
    coords::Vector{Tuple{Float64, Float64}}
    demand::Vector{Int64}
    t::Matrix{Float64}
end

function teste_instance()
    return instance_VRP(
        5,
        2,
        15,
        [(0.0, 0.0), (1.0, 3.0), (4.0, 4.0), (6.0, 2.0), (5.0, 0.0)],
        [0, 5, 10, 8, 7],             
        [0.0 10.0 15.0 20.0 25.0;     
         10.0 0.0 35.0 25.0 30.0;
         15.0 35.0 0.0 30.0 20.0;
         20.0 25.0 30.0 0.0 15.0;
         25.0 30.0 20.0 15.0 0.0]
    )

end



function open_demands(file::String)
    d1 = Int64[]
    open(file, "r") do io
        in_demand_section = false
        for line in eachline(io)
            if occursin("DEMAND_SECTION", line)
                in_demand_section = true
            elseif occursin("DEPOT_SECTION", line)
                break
            elseif in_demand_section
                parts = split(line)
                if length(parts) >= 2
                    push!(d1, parse(Int, parts[2]))
                else
                    throw(ArgumentError("Invalid Format"))
                end
            end
        end
    end

    if isempty(d1)
        throw(ArgumentError("Seccion not found"))
    end
    return d1
end

function calculate_costs(coords)
    num_points = length(coords)
    costs = zeros(Float64, num_points, num_points)
    for i in 1:num_points
        for j in 1:num_points
            costs[i, j] = sqrt((coords[i][1] - coords[j][1])^2 + (coords[i][2] - coords[j][2])^2)
        end
    end
    return costs
end

function test_instance(instance, n1, Q1, k1, coords1::Vector{Tuple{Int64, Int64}})
    @test instance.n == n1
    @test instance.Q == Q1
    @test instance.K == k1
    @test instance.coords == coords1
end

function open_archive(file::String)
    coords1 = Vector{Tuple{Int64, Int64}}()
    n1, Q1, k1 = 0, 0, 0

    open(file, "r") do io
        in_coord_section = false
        for line in eachline(io)
            if occursin("DIMENSION", line)
                n1 = parse(Int, split(line)[end])
            elseif occursin("CAPACITY", line)
                Q1 = parse(Int, split(line)[end])
            elseif occursin("NODE_COORD_SECTION", line)
                in_coord_section = true
            elseif occursin("DEMAND_SECTION", line)
                break
            elseif in_coord_section
                coords_parts = split(line)
                if length(coords_parts) >= 3
                    push!(coords1, (parse(Int, coords_parts[2]), parse(Int, coords_parts[3])))
                else
                    throw(ArgumentError("Invalid Format"))
                end
            end
        end
    end

    if n1 == 0 || Q1 == 0 || isempty(coords1)
        throw(ArgumentError("File does not contain instructions or incorrect format"))
    end

    d1 = open_demands(file)
    total_demand = sum(d1[2:end])
    k1 = ceil(Int, total_demand / Q1)
    c1 = calculate_costs(coords1)

    return coords1, VRP_instances(n1, k1, Q1, coords1, d1, c1)
end