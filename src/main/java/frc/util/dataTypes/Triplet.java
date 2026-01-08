package frc.util.dataTypes;

public class Triplet<K, V, A> extends Object { 
    public final K k;
    public final V v;
    public final A a;

    public Triplet(K k, V v, A a) { 
        this.k = k; 
        this.v = v; 
        this.a = a;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Triplet<?, ?, ?> other = (Triplet<?, ?, ?>) obj;
        return k.equals(other.k) && v.equals(other.v);
    }

    @Override
    public int hashCode() {
        return k.hashCode() ^ v.hashCode() ^ a.hashCode();
    }
  } 