package frc.util.dataTypes;

public class Tuple<K, V> extends Object { 
    public final K k;
    public final V v;
    public Tuple(K k, V v) { 
        this.k = k; 
        this.v = v; 
    } 

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Tuple<?, ?> other = (Tuple<?, ?>) obj;
        return k.equals(other.k) && v.equals(other.v);
    }

    @Override
    public int hashCode() {
        return k.hashCode() ^ v.hashCode();
    }
  } 