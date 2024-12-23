
class Solution {
public:
    int findMinMoves(vector<int>& machines) {
    //     int total = 0;
    //     for (int i = 0; i<machines.size(); i++) {
    //         total += machines[i];
    //     }

    //     if (total % machines.size() != 0) {
    //         return -1;
    //     }

    //     int target = total / machines.size();
    //     vector <int> moves(machines.size());
    //     int result = 0;

    //     for (int i = 0; i < machines.size()-1; i++) {
    //         int delta = machines[i] - target;
    //         machines[i+1] += delta;

    //         if (delta > 0) {
    //             moves[i] += delta;
    //         }
    //         else {
    //             moves[i+1] -= delta;
    //         }

    //         result = max(result, moves[i]);
    //     }

    //     result = max(result, machines.back() - target + moves.back());

    //     return result;


            int total = 0;
            for (int i = 0; i < machines.size(); i++) {
                total += machines[i];
            }

            
            if (total % machines.size() != 0) {
                return -1;
            }

            int target = total / machines.size(); 
            int result = 0;                      
            int differencies = 0;                  

            for (int i = 0; i < machines.size(); i++) {
                int delta = machines[i] - target; 
                differencies += delta;             
                result = max(result, max(abs(differencies), delta));
            }

            return result;
    }
};