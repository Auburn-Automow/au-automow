namespace ros {
    int fputc(char c, FILE *stream) {
        Serial.write(c);
        return 0;
    }
}
