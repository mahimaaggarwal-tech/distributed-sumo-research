#include <mpi.h>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

constexpr int kVehicleSlots = 4;

struct VehicleState {
    int active;
    int vehicle_id;
    int lane_id;
    double position_m;
    double speed_mps;
};

void check_mpi(int code, const char* what) {
    if (code != MPI_SUCCESS) {
        std::cerr << "MPI call failed: " << what << '\n';
        MPI_Abort(MPI_COMM_WORLD, code);
    }
}

}  // namespace

int main(int argc, char** argv) {
    check_mpi(MPI_Init(&argc, &argv), "MPI_Init");

    int world_rank = -1;
    int world_size = 0;
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);

    if (world_size < 2) {
        if (world_rank == 0) {
            std::cerr << "Run with at least two MPI ranks: mpirun -np 2 ./shared_vehicle_state_demo\n";
        }
        MPI_Finalize();
        return EXIT_FAILURE;
    }

    MPI_Comm shared_comm = MPI_COMM_NULL;
    check_mpi(MPI_Comm_split_type(MPI_COMM_WORLD, MPI_COMM_TYPE_SHARED,
                                  world_rank, MPI_INFO_NULL, &shared_comm),
              "MPI_Comm_split_type");

    int local_rank = -1;
    int local_size = 0;
    MPI_Comm_rank(shared_comm, &local_rank);
    MPI_Comm_size(shared_comm, &local_size);

    if (local_size < 2) {
        if (world_rank == 0) {
            std::cerr << "This demo needs at least two ranks on the same shared-memory node.\n";
        }
        MPI_Comm_free(&shared_comm);
        MPI_Finalize();
        return EXIT_FAILURE;
    }

    const MPI_Aint bytes = (local_rank == 0)
        ? static_cast<MPI_Aint>(kVehicleSlots * sizeof(VehicleState))
        : 0;
    void* base_ptr = nullptr;
    MPI_Win window = MPI_WIN_NULL;
    check_mpi(MPI_Win_allocate_shared(bytes, sizeof(char), MPI_INFO_NULL,
                                      shared_comm, &base_ptr, &window),
              "MPI_Win_allocate_shared");

    VehicleState* vehicles = nullptr;
    MPI_Aint queried_bytes = 0;
    int disp_unit = 0;
    void* queried_ptr = nullptr;
    check_mpi(MPI_Win_shared_query(window, 0, &queried_bytes,
                                   &disp_unit, &queried_ptr),
              "MPI_Win_shared_query");
    vehicles = static_cast<VehicleState*>(queried_ptr);

    if (local_rank == 0) {
        for (int i = 0; i < kVehicleSlots; ++i) {
            vehicles[i] = VehicleState{0, -1, -1, 0.0, 0.0};
        }
        vehicles[0] = VehicleState{1, 101, 7, 125.5, 13.2};
        vehicles[1] = VehicleState{1, 202, 8,  64.0,  8.7};
        check_mpi(MPI_Win_sync(window), "MPI_Win_sync after write");
        std::cout << "world rank " << world_rank
                  << " (shared rank 0) published two vehicle records\n";
    }

    check_mpi(MPI_Barrier(shared_comm), "MPI_Barrier after publish");
    check_mpi(MPI_Win_sync(window), "MPI_Win_sync before read");

    if (local_rank == 1) {
        std::cout << std::fixed << std::setprecision(1);
        for (int i = 0; i < kVehicleSlots; ++i) {
            if (vehicles[i].active) {
                std::cout << "world rank " << world_rank
                          << " read vehicle=" << vehicles[i].vehicle_id
                          << " lane=" << vehicles[i].lane_id
                          << " position_m=" << vehicles[i].position_m
                          << " speed_mps=" << vehicles[i].speed_mps << '\n';
            }
        }
    }

    check_mpi(MPI_Barrier(shared_comm), "MPI_Barrier before cleanup");
    check_mpi(MPI_Win_free(&window), "MPI_Win_free");
    check_mpi(MPI_Comm_free(&shared_comm), "MPI_Comm_free");
    MPI_Finalize();
    return EXIT_SUCCESS;
}
