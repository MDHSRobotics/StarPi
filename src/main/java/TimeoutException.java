

public class TimeoutException extends Exception {

    private final String reason;
    
    public TimeoutException(String reason) {
        this.reason = reason;
    }
    
    @Override
    public String toString() {
        return this.reason;
    }

}
