#include <Arduino.h>
#include "config.hpp"
#include "buzzer.hpp"
#include "scheduler.hpp"

#include <string.h>
#include <math.h>

long buzzer = PIN_BUZZER;
int beep_number();
void setbeep_number(long *arr);
void value_change();
void beep_loop();

int size = 3;
long *inputs;
int x=0; //position of array
int num_length; //power of the number
long num_current; //current value of whole number inputted, gets changed in number_Available. 153 -> 15 -> 1
long i = 0; //rightmost digit
bool next = true; //true if needs to check to be buzzed false for delay
bool firstcall = false;  //first call of focused value of array
bool is_zero = true;

void buzzer_setup(long *arr) {
  pinMode (buzzer, OUTPUT);
  setbeep_number(arr);
  scheduler_add(TaskId::Beep, Task(beep_loop, 100*1000L, 100));
}

void setbeep_number(long *arr){

inputs = arr;

/*//prints them for visual effect
  for (int g = 0; g < size; g++){
    Serial.print(inputs[g]);Serial.print("\n");
  }
*/
//compute on index 0
  num_length = log10(inputs[0]);
  num_length = pow(10,num_length) + .5;

  num_current = inputs[0];
  i = num_current / num_length;
}

void beep_loop(){
  value_change();
  beep_number();
}

void value_change(){//goes to next digit in array
  
  if (firstcall){
    if (x != size){
      x++;
    }
    
    num_length = log10(inputs[x]);
    num_length = pow(10,num_length) + .5;

    num_current = inputs[x];
    i = num_current / num_length;
    firstcall = false;
  }
}

int beep_number(){
  
  if(inputs[x] != NULL && inputs[x] >= 0 && num_length > 0){
    if (num_current == 0 && num_length == 0){
        firstcall = true;
    }

    if(next){
        next = !next;
        
        if (num_current / num_length == 0 && is_zero){
        i = 10;
        is_zero = false;
        //Serial.print("Changed i to 10\n");
        }
        
        if (i<=0){   //current number has been entirely buzzed and needs to move on to next number

        if(num_current % num_length != 0){
            num_current %= num_length;
        }
        else {
            num_current = 0;
        }
        num_length /= 10;
        i = num_current / num_length;
        is_zero = true;
        digitalWrite(buzzer,LOW);

        if (num_length==0){
            firstcall=true;
        }
        return 1;      
        }
        else{   //current number still needs to be buzzed
        // Serial.print(num_current);Serial.print(": ");Serial.print(i);Serial.print(" ");

        i--;                            //deduct 1 from current number          
        // Serial.println("\nBeep");
        digitalWrite(buzzer,HIGH);     
        return 0;
        }
        
    }
    else {
        next = !next;

        digitalWrite(buzzer,LOW);
        delay(100);
        return 3;
    }
  }
}