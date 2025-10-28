package org.firstinspires.ftc.teamcode;

public class Pratice {
    public static void main(String[] args) {
            System.out.println(calc(1, '+', 2));
            System.out.println(calc(5, '-', 3));
            System.out.println(calc(2, '*', 3));
            System.out.println(calc(20, '/', 5));
        }

    public static int calc(int num1, char character, int num2) {
        if (character == '+'){
            return num1 + num2;
        } else if (character == '-'){
            return num1 - num2;
        } else if (character == '*'){
            return num1 * num2;
        } else if (character == '/'){
            return num1 / num2;
        } else{
            return 0;
        }
    }
}

