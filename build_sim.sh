docker buildx build --build-arg max_step_size=0.001 --build-arg physics_type=ode --tag lucasmrdt/sailboat-sim-lsa-gym:mss1-ode --cache-from type=registry,ref=lucasmrdt/sailboat-sim-lsa-gym .

# https://www.squash.io/preventing-terminal-print-from-bash-scripts-in-linux/#:~:text=One%20way%20to%20suppress%20the,%2Fdev%2Fnull%E2%80%9D).
docker container stop sailboat-sim-lsa-gym-default > /dev/null 2>&1

echo "Attempting to remove unused docker images to free up space on your PC"
docker image prune