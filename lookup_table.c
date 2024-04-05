#include <stdio.h>
#include <math.h>
#include <string.h>

enum Finger{
    thumb, //0
    //index,
    middle,
    ring,
    little //4
};

enum Interdigital {
    thumb_index, //0
    index_middle,
    middle_ring,
    ring_little //3
};

struct Gesture {
    //finger curve
    int thumb_curve;
    int index_curve; 
    int middle_curve;
    int ring_curve;
    int little_curve;
    //finger touch 
    int thumb_index_touch;
    int index_middle_touch;
    int middle_ring_touch;
    int ring_little_touch; 

};

struct Gesture A_gesture = {0,1,1,1,1,   1,1,1,1};
struct Gesture B_gesture = {1,0,0,0,0,   0,1,1,1};
struct Gesture C_gesture = {1,1,1,1,1,   0,1,1,1};
struct Gesture D_gesture = {1,0,1,1,1,   0,0,1,1};
struct Gesture E_gesture = {1,1,1,1,1,   1,1,1,1};//E and C --> in order to break the tie, we may need to add a muti angle and space conductive senser between the thumb and index finger. 
struct Gesture F_gesture = {1,1,0,0,0,   1,0,1,1};
struct Gesture G_gesture = {0,0,1,1,1,   1,0,1,1};//add thumb and index finger touch detection to break the tie 
struct Gesture H_gesture = {1,0,0,1,1,   0,1,0,1};//may cause tie since we don't detect if the thumb touches ring finger 
struct Gesture I_gesture = {1,1,1,1,0,   1,1,1,0};
struct Gesture J_gesture = {1,1,1,1,0,   1,1,1,0};//right now we must have a tie between the I and J 
struct Gesture K_gesture = {1,0,0,1,1,   0,0,0,1};
struct Gesture L_gesture = {0,0,1,1,1,   0,0,1,1};
struct Gesture M_gesture = {1,1,1,1,1,   1,1,1,1};//have tie with E
struct Gesture N_gesture = {1,1,1,1,1,   1,1,1,1};//have tie with M --> need more sensor \addindex 
struct Gesture O_gesture = {1,1,1,1,1,   1,1,1,1};//have tie with E
struct Gesture P_gesture = {0,0,0,1,1,   0,0,0,1}; 
struct Gesture Q_gesture = {0,0,1,1,1,   0,0,1,1};
struct Gesture R_gesture = {1,0,0,1,1,   0,1,0,1};
struct Gesture S_gesture = {1,1,1,1,1,   1,1,1,1};
struct Gesture T_gesture = {1,1,1,1,1,   1,1,1,1};
struct Gesture U_gesture = {1,0,0,1,1,   0,1,0,1};
struct Gesture V_gesture = {1,0,0,1,1,   0,0,0,1};
struct Gesture W_gesture = {1,0,0,0,1,   0,0,0,0};
struct Gesture X_gesture = {1,1,1,1,1,   0,0,1,1};
struct Gesture Y_gesture = {0,1,1,1,0,   0,1,1,0};
struct Gesture Z_gesture = {1,0,1,1,1,   0,0,1,1};

int calculate_unique_index (struct Gesture gesture_temp){
    int result = gesture_temp.thumb_curve * pow(2,0) + 
    gesture_temp.index_curve * pow(2,1) + 
    gesture_temp.middle_curve * pow(2,2) + 
    gesture_temp.ring_curve * pow(2,3) + 
    gesture_temp.little_curve * pow(2,4) + 
   
    gesture_temp.thumb_index_touch * pow(2,5) + 
    gesture_temp.index_middle_touch * pow(2,6) + 
    gesture_temp.middle_ring_touch * pow(2,7) + 
    gesture_temp.ring_little_touch * pow(2,8);
   return result; 
}

//-------------------------------------------------------------------
// Define a struct for key-value pair
#define MAX_VALUES_PER_KEY 30
#define MAX_KEY_LENGTH 30
#define MAX_VALUE_LENGTH 30

struct KeyValuePair {
    int key;
    char values[MAX_VALUES_PER_KEY][MAX_VALUE_LENGTH];
    int num_values;
};

// Function to add a value to the dictionary
void add_value(struct KeyValuePair *dictionary, int key, const char *value) {
    for (int i = 0; i < MAX_VALUES_PER_KEY; i++) {
        if (dictionary[i].key == key) {
            strncpy(dictionary[i].values[dictionary[i].num_values++], value, MAX_VALUE_LENGTH);
            return;
        }
    }

    // Key not found, add a new entry
     for (int i = 0; i < MAX_VALUES_PER_KEY; i++) {
        if (dictionary[i].num_values == 0) {
            dictionary[i].key = key;
            strncpy(dictionary[i].values[dictionary[i].num_values++], value, MAX_VALUE_LENGTH - 1);
            return;
        }
    }

    // No space left for new keys
    printf("Dictionary full, cannot add more keys\n");
}

// Function to print the values associated with a key
void print_values(struct KeyValuePair *dictionary, int key) {
    for (int i = 0; i < MAX_VALUES_PER_KEY; i++) {
        if (dictionary[i].key == key) {
            printf("Values for key '%d': ", key);
            for (int j = 0; j < dictionary[i].num_values; j++) {
                printf("%s ", dictionary[i].values[j]);
            }
            printf("\n");
            return;
        }
    }

    printf("Key '%d' not found in dictionary\n", key);
}

void print_dictionary(struct KeyValuePair *dictionary) {
    for (int i = 0; i < MAX_VALUES_PER_KEY; i++) {
        if (dictionary[i].num_values > 0) {
            printf("Values for key '%d': ", dictionary[i].key);
            for (int j = 0; j < dictionary[i].num_values; j++) {
                printf("%s ", dictionary[i].values[j]);
            }
            printf("\n");
        }
    }
}

int main(){
    printf("%d\n", thumb); 
    struct Gesture bag[26] = { A_gesture, B_gesture, C_gesture, D_gesture, E_gesture, F_gesture,
                           G_gesture, H_gesture, I_gesture, J_gesture, K_gesture, L_gesture,
                           M_gesture, N_gesture, O_gesture, P_gesture, Q_gesture, R_gesture,
                           S_gesture, T_gesture, U_gesture, V_gesture, W_gesture, X_gesture,
                           Y_gesture, Z_gesture };
    struct KeyValuePair dictionary[MAX_VALUES_PER_KEY] = {0};
    char start = 'A';
    for (int i = 0; i < 26; i++){
        printf("the unique size of %c is: %i \n", start, calculate_unique_index(bag[i]));
        add_value(dictionary,calculate_unique_index(bag[i]), &start);
        start++;
    }

    print_dictionary(dictionary);

    //currently we have problem: 
    // D Z solving solution:
        // 1. use the accelerometer to detect if the hand move 
        // 2. create more level detection for the finger curve --> stright, half cruve, total curve 
    // E M O S T N 
            // E: thumb touch the outside of the four finger
                                                        // ----> we need to have one more sensor place 
            // M: thum touch the inside of the four finger 


            //O: four fingers are half curve & the thum don't touch the inside or outside of four finger

            //S: the position of the thumb touchs with the middle finger ---> we need to have one more sensor place 

            //T: the thumb above only touch with the inside of the index finger ---> we need to have one more sensor place above thumb 

            //N : the thumb above touch with the middle and index inside. 

    // H R U 
    // K V 
    // L Q

    
    return 0;
}